// Generic UART-programmable USB device engine. PS3 policy lives in host
// profiles.
#include "class/hid/hid.h"
#include "device/usbd_pvt.h"
#include "hardware/irq.h"
#include "hardware/uart.h"
#include "pico/bootrom.h"
#include "pico/stdlib.h"
#include "tusb.h"
#include <string.h>
#define NIF 4
#define MAXPAY 2048
#define RING 8192
static uint8_t rx[RING], tx[RING];
static volatile uint32_t rh, rt;
static uint32_t th, tt, dropped, rx_dropped;
static uint8_t dev[18], config[512], reports[NIF][1024], strings[16][128];
static uint16_t clen, rlen[NIF];
static bool attached;
static bool timing_events;
static uint8_t inep[NIF], outep[NIF], hid_desc[NIF][9], outsize[NIF],
    outbuf[NIF][64], usb_in[NIF][64];
static uint8_t current[NIF][64], neutral[NIF][64], ilen[NIF], nlen[NIF];
static uint32_t updated[NIF], timeout_ms = 1000;
static uint8_t ctrlbuf[512];
typedef struct {
  uint8_t match[8], mask[8], action;
  uint16_t len;
  uint8_t data[512];
} rule_t;
static rule_t rules[32];
static uint8_t nrules;
static int selected = -1;
static uint16_t u16(const uint8_t *p) { return p[0] | ((uint16_t)p[1] << 8); }
static uint16_t crc16(const uint8_t *p, unsigned n) {
  uint16_t c = 0xffff;
  while (n--) {
    c ^= (uint16_t)*p++ << 8;
    for (int j = 0; j < 8; j++)
      c = (c & 0x8000) ? (c << 1) ^ 0x1021 : c << 1;
  }
  return c;
}
static void uart_rx(void) {
  while (uart_is_readable(uart0)) {
    uint8_t b = uart_getc(uart0);
    if (rh - rt < RING)
      rx[rh++ & (RING - 1)] = b;
    else
      rx_dropped++;
  }
}
static void emit(uint8_t op, uint16_t seq, const void *p, uint16_t n) {
  uint8_t b[MAXPAY + 10];
  if (n > MAXPAY)
    return;
  b[0] = 0xa5;
  b[1] = 0x5a;
  b[2] = 1;
  b[3] = op;
  b[4] = seq;
  b[5] = seq >> 8;
  b[6] = n;
  b[7] = n >> 8;
  memcpy(b + 8, p, n);
  uint16_t c = crc16(b + 2, n + 6);
  b[n + 8] = c;
  b[n + 9] = c >> 8;
  if (th - tt + n + 10 > RING) {
    dropped++;
    return;
  }
  for (unsigned i = 0; i < n + 10u; i++)
    tx[th++ & (RING - 1)] = b[i];
}
static void event(uint8_t type, const void *p, uint16_t n) {
  uint8_t b[521];
  if (n > 520)
    n = 520;
  b[0] = type;
  memcpy(b + 1, p, n);
  emit(0x90, 0, b, n + 1);
}
static void reset(uint8_t port) {
  (void)port;
  memset(inep, 0, sizeof inep);
  memset(outep, 0, sizeof outep);
  memset(hid_desc, 0, sizeof hid_desc);
  event(1, NULL, 0);
}
static void init(void) {}
static bool deinit(void) { return true; }
static uint16_t open_itf(uint8_t port, tusb_desc_interface_t const *itf,
                         uint16_t max) {
  if (itf->bInterfaceNumber >= NIF || itf->bAlternateSetting)
    return 0;
  uint8_t i = itf->bInterfaceNumber;
  uint8_t const *p = (const uint8_t *)itf;
  uint16_t pos = itf->bLength;
  while (pos + 2 <= max && p[pos + 1] != TUSB_DESC_INTERFACE) {
    uint8_t n = p[pos];
    if (n < 2 || pos + n > max)
      return 0;
    if (p[pos + 1] == HID_DESC_TYPE_HID && n == 9)
      memcpy(hid_desc[i], p + pos, 9);
    if (p[pos + 1] == TUSB_DESC_ENDPOINT && n >= 7) {
      tusb_desc_endpoint_t const *e = (const void *)(p + pos);
      if (tu_edpt_packet_size(e) > 64 || !usbd_edpt_open(port, e))
        return 0;
      if (e->bEndpointAddress & 0x80)
        inep[i] = e->bEndpointAddress;
      else {
        outep[i] = e->bEndpointAddress;
        outsize[i] = tu_edpt_packet_size(e);
        usbd_edpt_xfer(port, outep[i], outbuf[i], tu_edpt_packet_size(e));
      }
    }
    pos += n;
  }
  return pos;
}
static bool control(uint8_t port, uint8_t stage,
                    tusb_control_request_t const *r) {
  uint8_t i = r->wIndex & 255;
  if (stage == CONTROL_STAGE_SETUP) {
    selected = -1;
    event(2, r, 8);
    for (int k = 0; k < nrules; k++) {
      bool yes = true;
      for (int j = 0; j < 8; j++)
        if ((((const uint8_t *)r)[j] & rules[k].mask[j]) !=
            (rules[k].match[j] & rules[k].mask[j]))
          yes = false;
      if (!yes)
        continue;
      selected = k;
      rule_t *q = &rules[k];
      if (q->action == 0)
        return false;
      if (q->action == 1) {
        memcpy(ctrlbuf, q->data, q->len);
        return tud_control_xfer(port, r, ctrlbuf, q->len);
      }
      if (q->action == 2 || q->action == 4) {
        if (r->bmRequestType_bit.direction != TUSB_DIR_OUT)
          return false;
        if (r->wLength > sizeof ctrlbuf)
          return false;
        return tud_control_xfer(port, r, ctrlbuf, r->wLength);
      }
      if (q->action == 3)
        return tud_control_status(port, r);
    }
    if (i < NIF && r->bmRequestType == 0x81 &&
        r->bRequest == TUSB_REQ_GET_DESCRIPTOR) {
      if ((r->wValue >> 8) == HID_DESC_TYPE_REPORT)
        return tud_control_xfer(port, r, reports[i], rlen[i]);
      if ((r->wValue >> 8) == HID_DESC_TYPE_HID)
        return tud_control_xfer(port, r, hid_desc[i], 9);
    }
    if (i < NIF && r->bmRequestType == 0xa1 &&
        r->bRequest == HID_REQ_CONTROL_GET_REPORT && (r->wValue >> 8) == 1) {
      memcpy(ctrlbuf, current[i], ilen[i]);
      return tud_control_xfer(port, r, ctrlbuf, ilen[i]);
    }
    if (i < NIF && r->bmRequestType == 0x21 &&
        r->bRequest == HID_REQ_CONTROL_SET_REPORT) {
      if (r->wLength > sizeof ctrlbuf)
        return false;
      return tud_control_xfer(port, r, ctrlbuf, r->wLength);
    }
    if (i < NIF && r->bmRequestType == 0x21 &&
        (r->bRequest == HID_REQ_CONTROL_SET_IDLE ||
         r->bRequest == HID_REQ_CONTROL_SET_PROTOCOL))
      return tud_control_status(port, r);
    event(3, r, 8);
    return false;
  }
  if (stage == CONTROL_STAGE_ACK &&
      r->bmRequestType_bit.direction == TUSB_DIR_OUT && r->wLength) {
    uint8_t b[520];
    memcpy(b, r, 8);
    uint16_t n = r->wLength > 512 ? 512 : r->wLength;
    memcpy(b + 8, ctrlbuf, n);
    event(4, b, n + 8);
    if (selected >= 0 && rules[selected].action == 4) {
      rule_t *q = &rules[selected];
      if (q->len == 4) {
        unsigned target = q->data[0], src = q->data[1], dst = q->data[2],
                 count = q->data[3];
        if (target < nrules && src + count <= n &&
            dst + count <= rules[target].len)
          memcpy(rules[target].data + dst, ctrlbuf + src, count);
      }
    }
    if (selected >= 0 && rules[selected].action == 2) {
      rules[selected].len = n;
      memcpy(rules[selected].data, ctrlbuf, n);
    }
  }
  return true;
}
static bool xfer(uint8_t p, uint8_t ep, xfer_result_t result, uint32_t n) {
  (void)result;
  for (int i = 0; i < NIF; i++)
    if (ep == outep[i]) {
      uint8_t b[65];
      b[0] = i;
      memcpy(b + 1, outbuf[i], n > 64 ? 64 : n);
      event(5, b, (n > 64 ? 64 : n) + 1);
      usbd_edpt_xfer(p, ep, outbuf[i], outsize[i]);
    }
  return true;
}
static usbd_class_driver_t const driver = {.name = "Generic",
                                           .init = init,
                                           .deinit = deinit,
                                           .reset = reset,
                                           .open = open_itf,
                                           .control_xfer_cb = control,
                                           .xfer_cb = xfer};
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *n) {
  *n = 1;
  return &driver;
}
bool tud_vendor_control_xfer_cb(uint8_t p, uint8_t s,
                                tusb_control_request_t const *r) {
  return control(p, s, r);
}
uint8_t const *tud_descriptor_device_cb(void) { return dev; }
uint8_t const *tud_descriptor_configuration_cb(uint8_t i) {
  (void)i;
  return config;
}
uint16_t const *tud_descriptor_string_cb(uint8_t i, uint16_t lang) {
  (void)lang;
  return i < 16 && strings[i][0] ? (uint16_t *)strings[i] : NULL;
}
void tud_mount_cb(void) { event(6, NULL, 0); }
void tud_umount_cb(void) { event(7, NULL, 0); }
void tud_suspend_cb(bool wake) { event(8, &wake, 1); }
void tud_resume_cb(void) { event(9, NULL, 0); }
static bool valid(void) {
  if (dev[0] != 18 || dev[1] != 1 || dev[7] != 64 || dev[17] != 1 || clen < 9 ||
      config[0] != 9 || config[1] != 2 || u16(config + 2) != clen ||
      config[4] < 1 || config[4] > NIF)
    return false;
  unsigned pos = 9, ifs = 0;
  uint32_t eps = 0;
  int itf = -1;
  while (pos < clen) {
    if (pos + 2 > clen)
      return false;
    unsigned n = config[pos];
    if (n < 2 || pos + n > clen)
      return false;
    uint8_t t = config[pos + 1];
    if (t == 4) {
      if (ifs >= NIF || n != 9 || config[pos + 2] != ifs || config[pos + 3] ||
          !rlen[ifs])
        return false;
      itf = ifs++;
      if (ifs > NIF)
        return false;
    }
    if (t == 5) {
      if (n != 7 || itf < 0)
        return false;
      unsigned a = config[pos + 2], ep = a & 15,
               key = ep + ((a & 128) ? 16 : 0);
      if (!ep || (eps & (1u << key)) || u16(config + pos + 4) > 64 ||
          (config[pos + 3] & 3) != 3)
        return false;
      eps |= 1u << key;
    }
    pos += n;
  }
  return ifs == config[4];
}
static void command(uint8_t op, uint16_t seq, uint8_t *p, uint16_t n) {
  uint8_t err = 0;
  switch (op) {
  case 1: {
    uint32_t status[] = {1,          attached, tud_mounted(),
                         timeout_ms, dropped,  rx_dropped};
    emit(0x81, seq, status, sizeof status);
    return;
  }
  case 2:
    tud_disconnect();
    attached = false;
    sleep_ms(20);
    break;
  case 3:
    if (!valid()) {
      err = 3;
      break;
    }
    tud_disconnect();
    sleep_ms(30);
    tud_deinit(0);
    tud_init(0);
    tud_connect();
    attached = true;
    break;
  case 4:
    if (attached) {
      err = 2;
      break;
    }
    if (n < 2) {
      err = 1;
      break;
    }
    if (p[0] == 0 && n == 20)
      memcpy(dev, p + 2, 18);
    else if (p[0] == 1 && n - 2 <= 512) {
      clen = n - 2;
      memcpy(config, p + 2, clen);
    } else if (p[0] == 2 && p[1] < NIF && n - 2 <= 1024) {
      rlen[p[1]] = n - 2;
      memcpy(reports[p[1]], p + 2, n - 2);
    } else if (p[0] == 3 && p[1] < 16 && n >= 4 && n - 2 <= 128 &&
               p[2] == n - 2 && p[3] == 3)
      memcpy(strings[p[1]], p + 2, n - 2);
    else
      err = 1;
    break;
  case 5:
    if (attached) {
      err = 2;
      break;
    }
    nrules = 0;
    break;
  case 6:
    if (attached) {
      err = 2;
      break;
    }
    if (n < 19 || nrules == 32 || p[16] > 4 || u16(p + 17) > 512 ||
        n != 19 + u16(p + 17)) {
      err = 1;
      break;
    }
    memcpy(rules[nrules].match, p, 8);
    memcpy(rules[nrules].mask, p + 8, 8);
    rules[nrules].action = p[16];
    rules[nrules].len = u16(p + 17);
    memcpy(rules[nrules].data, p + 19, n - 19);
    nrules++;
    break;
  case 7:
  case 8:
    if (n < 2 || p[0] >= NIF || n > 65) {
      err = 1;
      break;
    }
    if (op == 7) {
      uint64_t received_us = time_us_64();
      memcpy(current[p[0]], p + 1, n - 1);
      ilen[p[0]] = n - 1;
      updated[p[0]] = to_ms_since_boot(get_absolute_time());
      if (timing_events) {
        uint8_t sample[75];
        sample[0] = seq;
        sample[1] = seq >> 8;
        memcpy(sample + 2, &received_us, 8);
        memcpy(sample + 10, p, n);
        event(0x20, sample, n + 10);
      }
    } else {
      memcpy(neutral[p[0]], p + 1, n - 1);
      nlen[p[0]] = n - 1;
    }
    break;
  case 9:
    if (n != 4) {
      err = 1;
      break;
    }
    memcpy(&timeout_ms, p, 4);
    break;
  case 10:
    if (n != 4 || memcmp(p, "BOOT", 4)) {
      err = 1;
      break;
    }
    reset_usb_boot(0, 0);
    break;
  case 11: {
    if (n != 0) { err = 1; break; }
    uint64_t now_us = time_us_64();
    emit(0x8b, seq, &now_us, sizeof now_us);
    return;
  }
  case 12:
    if (n != 1 || p[0] > 1) { err = 1; break; }
    timing_events = p[0] != 0;
    break;
  default:
    err = 1;
  }
  emit(0x80, seq, &err, 1);
}
int main(void) {
  uart_init(uart0, 1000000);
  gpio_set_function(0, GPIO_FUNC_UART);
  gpio_set_function(1, GPIO_FUNC_UART);
  uart_set_fifo_enabled(uart0, true);
  irq_set_exclusive_handler(UART0_IRQ, uart_rx);
  irq_set_enabled(UART0_IRQ, true);
  uart_set_irq_enables(uart0, true, false);
  tud_init(0);
  tud_disconnect();
  gpio_init(25);
  gpio_set_dir(25, GPIO_OUT);
  uint8_t packet[MAXPAY + 10];
  unsigned used = 0;
  uint32_t last = 0;
  while (1) {
    tud_task();
    while (tt != th && uart_is_writable(uart0))
      uart_putc_raw(uart0, tx[tt++ & (RING - 1)]);
    uint32_t now = to_ms_since_boot(get_absolute_time());
    if (used && now - last > 100)
      used = 0;
    while (rt != rh) {
      uint8_t b = rx[rt++ & (RING - 1)];
      last = now;
      if (used == 0 && b != 0xa5)
        continue;
      if (used == 1 && b != 0x5a) {
        used = b == 0xa5 ? 1 : 0;
        continue;
      }
      packet[used++] = b;
      if (used >= 8) {
        unsigned n = u16(packet + 6);
        if (packet[2] != 1 || n > MAXPAY) {
          used = 0;
          continue;
        }
        if (used == n + 10) {
          if (crc16(packet + 2, n + 6) == u16(packet + n + 8))
            command(packet[3], u16(packet + 4), packet + 8, n);
          used = 0;
        }
      }
    }
    now = to_ms_since_boot(get_absolute_time());
    for (int i = 0; i < NIF; i++) {
      if (timeout_ms && now - updated[i] > timeout_ms && nlen[i]) {
        memcpy(current[i], neutral[i], nlen[i]);
        ilen[i] = nlen[i];
      }
      if (attached && tud_mounted() && inep[i] && ilen[i] &&
          usbd_edpt_ready(0, inep[i])) {
        memcpy(usb_in[i], current[i], ilen[i]);
        usbd_edpt_xfer(0, inep[i], usb_in[i], ilen[i]);
      }
    }
    gpio_put(25, attached ? tud_mounted() : ((now / 500) & 1));
  }
}
