#!/usr/bin/env python3
"""Forward Guide press edges to webMAN without touching UART or delaying game input."""
import argparse
import fcntl
import http.client
import os
import select
import signal
import threading
import time
from evdev_relay import EVENT, discover

GUIDE = 316

class GuideSender:
    def __init__(self, host):
        self.host = host
        self.busy = threading.Lock()

    def press(self):
        # Never queue/retry a stale PS press after a network outage.
        if not self.busy.acquire(blocking=False):
            return False
        threading.Thread(target=self._send, daemon=True).start()
        return True

    def _send(self):
        conn = http.client.HTTPConnection(self.host, timeout=2)
        try:
            conn.request('GET', '/pad.ps3?psbtn')
            response = conn.getresponse()
            if response.status != 200:
                raise RuntimeError(f'HTTP {response.status}')
            print('Guide sent to webMAN', flush=True)
        except (OSError, http.client.HTTPException, RuntimeError) as error:
            print(f'Guide failed (not retried): {error}', flush=True)
        finally:
            conn.close()
            self.busy.release()


def held(fd):
    keys = bytearray(96)
    fcntl.ioctl(fd, 0x80604518, keys, True)  # EVIOCGKEY(96)
    return bool(keys[GUIDE // 8] & (1 << (GUIDE % 8)))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True)
    args = parser.parse_args()
    sender = GuideSender(args.host)
    devices = {}
    next_scan = 0
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        while True:
            if time.monotonic() >= next_scan:
                next_scan = time.monotonic() + .5
                paths = set(discover())
                opened = {state[0] for state in devices.values()}
                for path in paths - opened:
                    fd = None
                    try:
                        fd = os.open(path, os.O_RDONLY | os.O_NONBLOCK)
                        devices[fd] = [path, held(fd), False]
                    except OSError:
                        if fd is not None:
                            os.close(fd)
                        continue
            for fd in select.select(list(devices), [], [], .1)[0]:
                try:
                    data = os.read(fd, EVENT.size * 64)
                except BlockingIOError:
                    continue
                except OSError:
                    data = b''
                if not data:
                    os.close(fd)
                    del devices[fd]
                    continue
                state = devices[fd]
                for offset in range(0, len(data), EVENT.size):
                    _, _, kind, code, value = EVENT.unpack_from(data, offset)
                    if kind == 0 and code == 3:
                        state[2] = True
                    elif state[2]:
                        if kind == 0 and code == 0:
                            state[1] = held(fd)
                            state[2] = False
                    elif kind == 1 and code == GUIDE:
                        if value == 1 and not state[1]:
                            sender.press()
                        if value != 2:
                            state[1] = bool(value)
    except KeyboardInterrupt:
        pass
    finally:
        for fd in devices:
            os.close(fd)

if __name__ == '__main__':
    main()
