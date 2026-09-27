#!/usr/bin/env python3
"""Relay Sunshine x360 evdev controllers to the generic UART engine.

Usage: evdev_relay.py --port /dev/ttyUSB0 /dev/input/eventN [...]
With no event arguments, discover Sunshine-compatible Xbox 360 virtual pads.
Loads the matching ps3-N.json profile and restores it after Pico power cycles.
This process must be the sole UART owner. SIGINT/SIGTERM release all buttons.
"""
import argparse
import glob
import json
import os
import select
import signal
import subprocess
import re
import struct
import time
from pathlib import Path
from host import Engine, ps3_report, dinput_report

EVENT = struct.Struct('@llHHi')
# Sunshine's Xbox backend uses BTN_X=307 and BTN_Y=308. Their Linux
# NORTH/WEST aliases do not describe their positions on this virtual pad.
KEYS = {304:'cross', 305:'circle', 307:'square', 308:'triangle',
        310:'l1', 311:'r1', 314:'select', 315:'start', 316:'ps',
        317:'l3', 318:'r3'}

class Pad:
    def __init__(self, report_format='ps3'):
        self.report_format=report_format
        self.buttons = set()
        self.axes = {0:0, 1:0, 3:0, 4:0, 2:0, 5:0, 16:0, 17:0}
    def event(self, kind, code, value):
        if kind == 1 and code in KEYS:
            if value: self.buttons.add(KEYS[code])
            else: self.buttons.discard(KEYS[code])
        elif kind == 3: self.axes[code] = value
    def report(self):
        buttons = set(self.buttons)
        for axis, neg, pos in ((16,'left','right'), (17,'up','down')):
            if self.axes[axis] < 0: buttons.add(neg)
            elif self.axes[axis] > 0: buttons.add(pos)
        for axis, button in ((2,'l2'), (5,'r2')):
            if self.axes[axis] > 0: buttons.add(button)
        def stick(axis): return max(0,min(255,(self.axes[axis]+32768)*256//65536))
        encode = dinput_report if self.report_format=='dinput' else ps3_report
        report = bytearray(encode(buttons, stick(0),stick(1),stick(3),stick(4)))
        left,right = (17,18) if self.report_format=='dinput' else (18,19)
        report[left] = max(0,min(255,self.axes[2]))
        report[right] = max(0,min(255,self.axes[5]))
        return bytes(report)

def is_gamepad(path):
    event=Path('/sys/class/input',Path(path).name)
    try:
        name=(event/'device/name').read_text().strip()
        absolute=int((event/'device/capabilities/abs').read_text().split()[-1],16)
    except (OSError,ValueError,IndexError): return False
    return absolute & 3 == 3 and ('Sunshine' in name or name == 'Microsoft X-Box 360 pad')

def discover():
    return ['/dev/input/'+Path(p).name for p in sorted(glob.glob('/sys/class/input/event*'),key=lambda p:int(Path(p).name[5:])) if is_gamepad(p)]

class Hotplug:
    """One initial scan; thereafter only inspect nodes named by udev events."""
    def __init__(self):
        self.process=subprocess.Popen(['udevadm','monitor','--udev','--subsystem-match=input'],stdout=subprocess.PIPE,stderr=subprocess.DEVNULL)
        self.fd=self.process.stdout.fileno();os.set_blocking(self.fd,False)
        self.buffer=b'';self.paths=set(discover())
    def read(self):
        data=os.read(self.fd,16384)
        if not data: raise RuntimeError('udev hotplug monitor stopped')
        self.buffer+=data
        while b'\n' in self.buffer:
            line,self.buffer=self.buffer.split(b'\n',1)
            match=re.search(rb'\b(add|change|remove)\s+\S*/(event[0-9]+)\s',line)
            if not match: continue
            path='/dev/input/'+match[2].decode()
            if match[1]==b'remove': self.paths.discard(path)
            elif is_gamepad(path): self.paths.add(path)
    def close(self):
        self.process.terminate();self.process.wait();self.process.stdout.close()

def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port',default='/dev/ttyUSB0')
    parser.add_argument('--controllers',type=int,choices=(1,2,4),default=1)
    parser.add_argument('--profile')
    parser.add_argument('events',nargs='*')
    args=parser.parse_args()
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    engine=Engine(args.port)
    profile=json.loads(Path(args.profile or Path(__file__).with_name(f'ps3-{args.controllers}.json')).read_text())
    engine.load(profile)
    report_format=profile.get('report_format','ps3')
    neutral=dinput_report() if report_format=='dinput' else ps3_report()
    monitor=Hotplug()
    devices={}; slots={}; last_send=0; next_status=0; previous_status=None
    try:
        while True:
            now=time.monotonic()
            if now>=next_status:
                status=engine.status()
                if not status[1]:
                    engine.load(profile)
                    status=engine.status()
                if status!=previous_status:
                    print(f'Pico status (version, attached, mounted, timeout, event drops, rx drops): {status}',flush=True)
                    previous_status=status
                next_status=now+1
            candidates=args.events or sorted(monitor.paths,key=lambda p:int(Path(p).name[5:]))
            for path in candidates:
                if path in slots: continue
                free=next((i for i in range(args.controllers) if i not in slots.values()),None)
                if free is None: break
                try: fd=os.open(path,os.O_RDONLY|os.O_NONBLOCK)
                except OSError: continue
                slots[path]=free; devices[fd]=(path,free,Pad(report_format))
                print(f'{path} -> controller {free}',flush=True)
            ready,_,_=select.select([*devices,monitor.fd],[],[],.05)
            if monitor.fd in ready:
                monitor.read();ready.remove(monitor.fd)
            for fd in ready:
                path,slot,pad=devices[fd]
                try: data=os.read(fd,EVENT.size*64)
                except BlockingIOError: continue
                except OSError: data=b''
                if not data:
                    engine.report(slot,neutral)
                    os.close(fd); del devices[fd]; del slots[path]
                    continue
                for offset in range(0,len(data),EVENT.size):
                    _,_,kind,code,value=EVENT.unpack_from(data,offset)
                    if kind==0 and code==3:
                        # SYN_DROPPED: release rather than replay incomplete state.
                        pad.buttons.clear(); pad.axes=Pad().axes
                    else: pad.event(kind,code,value)
                    if kind==0 and code==0:
                        engine.report(slot,pad.report())
            # Refresh held state below firmware watchdog interval, including idle.
            if time.monotonic()-last_send>=.1:
                for _,slot,pad in devices.values(): engine.report(slot,pad.report())
                engine.events.clear()
                last_send=time.monotonic()
    except KeyboardInterrupt:
        pass
    finally:
        for fd,(_,slot,_) in list(devices.items()):
            try: engine.report(slot,neutral)
            finally: os.close(fd)
        monitor.close()
        engine.close()

if __name__=='__main__': main()
