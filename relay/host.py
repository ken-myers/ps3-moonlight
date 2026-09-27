#!/usr/bin/env python3
"""UART USB engine client; uses only Python's standard library."""
import argparse,binascii,json,os,select,struct,termios,time,pathlib

def frame(op,seq,p=b''):
 h=struct.pack('<BBHH',1,op,seq,len(p))+p
 return b'\xa5\x5a'+h+struct.pack('<H',binascii.crc_hqx(h,0xffff))

class Engine:
 def __init__(self,port='/dev/ttyUSB0'):
  self.trace=open(os.environ['PS3_TIMING_LOG'],'a',buffering=1) if os.environ.get('PS3_TIMING_LOG') else None
  self.fd=os.open(port,os.O_RDWR|os.O_NOCTTY|os.O_NONBLOCK);self.seq=0;self.buf=bytearray();self.events=[]
  a=termios.tcgetattr(self.fd);a[0]=a[1]=a[3]=0;a[2]=termios.CS8|termios.CREAD|termios.CLOCAL;a[4]=a[5]=termios.B1000000;a[6][termios.VMIN]=0;a[6][termios.VTIME]=0;termios.tcsetattr(self.fd,termios.TCSANOW,a)
  if self.trace:
   for _ in range(40):self.command(11)
   self.command(12,b'\1')
 def timing(self,kind,**values):
  if self.trace:self.trace.write(json.dumps(dict(kind=kind,host_ns=time.monotonic_ns(),**values))+'\n')
 def read(self,timeout=.2):
  deadline=time.monotonic()+timeout
  while True:
   k=self.buf.find(b'\xa5\x5a')
   if k<0:
    self.buf[:]=self.buf[-1:] if self.buf[-1:]==b'\xa5' else b''
   elif k:self.buf[:k]=b''
   if len(self.buf)>=8:
    ver,op,seq,n=struct.unpack_from('<BBHH',self.buf,2)
    if ver!=1 or n>2048:del self.buf[0];continue
    if len(self.buf)>=n+10:
     b=bytes(self.buf[:n+10]);del self.buf[:n+10]
     if binascii.crc_hqx(b[2:-2],0xffff)==struct.unpack('<H',b[-2:])[0]:return op,seq,b[8:-2]
     continue
   left=deadline-time.monotonic()
   if left<=0:return None
   if select.select([self.fd],[],[],left)[0]:self.buf+=os.read(self.fd,8192)
 def command(self,op,p=b'',timeout=2):
  self.seq=(self.seq+1)&65535;b=frame(op,self.seq,p);sent_ns=time.monotonic_ns()
  while b:
   select.select([],[self.fd],[],timeout);n=os.write(self.fd,b);b=b[n:]
  deadline=time.monotonic()+timeout
  while time.monotonic()<deadline:
   r=self.read(deadline-time.monotonic())
   if r is None:break
   if r[0]==0x90:
    self.events.append(r[2])
    if self.trace and len(r[2])>=12 and r[2][0]==32:
     seq,pico_us,interface=struct.unpack_from('<HQB',r[2],1)
     self.timing('pico_accept',seq=seq,pico_us=pico_us,interface=interface,report=r[2][12:].hex())
    continue
   if r[1]==self.seq:
    if r[0]==0x80 and r[2]!=b'\0':raise RuntimeError(f'Opcode {op} rejected: {r[2].hex()}')
    if self.trace and op in (7,11):self.timing('command',opcode=op,seq=self.seq,sent_ns=sent_ns,received_ns=time.monotonic_ns(),payload=p.hex(),response=r[2].hex())
    return r[2]
  raise TimeoutError(f'No UART reply for opcode {op}')
 def status(self):
  if self.trace:self.command(11)
  return struct.unpack('<6I',self.command(1))
 def load(self,profile):
  self.command(2);self.command(5)
  self.command(4,b'\0\0'+bytes.fromhex(profile['device']))
  self.command(4,b'\1\0'+bytes.fromhex(profile['configuration']))
  for kind,key in ((2,'reports'),(3,'strings')):
   for i,b in enumerate(profile[key]):self.command(4,bytes([kind,i])+bytes.fromhex(b))
  for r in profile['rules']:
   data=bytes.fromhex(r['data']);self.command(6,bytes.fromhex(r['match']+r['mask'])+bytes([r['action']])+struct.pack('<H',len(data))+data)
  for i,b in enumerate(profile['neutral']):
   report=bytes.fromhex(b);self.command(8,bytes([i])+report);self.report(i,report)
  self.command(9,struct.pack('<I',profile.get('timeout_ms',1000)));self.command(3)
 def report(self,interface,report):return self.command(7,bytes([interface])+report)
 def close(self):
  if self.trace:
   try:
    for _ in range(40):self.command(11)
    self.command(12,b'\0')
   except (OSError,TimeoutError):pass
   self.trace.close();self.trace=None
  os.close(self.fd)

def ps3_report(buttons=(),lx=128,ly=128,rx=128,ry=128):
 b=bytearray(49);b[0]=1;b[6:10]=bytes([lx,ly,rx,ry]);b[30:33]=b'\x02\x05\x10'
 mapping={'select':(2,0),'l3':(2,1),'r3':(2,2),'start':(2,3),'up':(2,4),'right':(2,5),'down':(2,6),'left':(2,7),'l2':(3,0),'r2':(3,1),'l1':(3,2),'r1':(3,3),'triangle':(3,4),'circle':(3,5),'cross':(3,6),'square':(3,7),'ps':(4,0)}
 pressure={'up':14,'right':15,'down':16,'left':17,'l2':18,'r2':19,'l1':20,'r1':21,'triangle':22,'circle':23,'cross':24,'square':25}
 for key in buttons:
  index,bit=mapping[key];b[index]|=1<<bit
  if key in pressure:b[pressure[key]]=255
 return bytes(b)

def dinput_report(buttons=(),lx=128,ly=128,rx=128,ry=128):
 """PS3-compatible 19-byte SHANWAN report (tusb_gamepad/MIT layout)."""
 b=bytearray([0,0,8,lx,ly,rx,ry])+bytearray(12)
 buttons=set(buttons)
 mapping={'square':0,'cross':1,'circle':2,'triangle':3,'l1':4,'r1':5,'l2':6,'r2':7,'select':8,'start':9,'l3':10,'r3':11,'ps':12}
 for key,bit in mapping.items():
  if key in buttons:b[bit//8]|=1<<(bit%8)
 x=int('right'in buttons)-int('left'in buttons);y=int('down'in buttons)-int('up'in buttons)
 b[2]={(0,-1):0,(1,-1):1,(1,0):2,(1,1):3,(0,1):4,(-1,1):5,(-1,0):6,(-1,-1):7,(0,0):8}[x,y]
 for index,key in enumerate(('right','left','up','down','triangle','circle','cross','square','l1','r1','l2','r2'),7):
  if key in buttons:b[index]=255
 return bytes(b)

def main():
 p=argparse.ArgumentParser();p.add_argument('--port',default='/dev/ttyUSB0');s=p.add_subparsers(dest='cmd',required=True)
 s.add_parser('status');l=s.add_parser('load');l.add_argument('profile');l=s.add_parser('press');l.add_argument('button');l.add_argument('--seconds',type=float,default=.25);l.add_argument('--interface',type=int,default=0);s.add_parser('monitor');s.add_parser('detach');s.add_parser('boot')
 a=p.parse_args();e=Engine(a.port)
 try:
  if a.cmd=='status':print(dict(zip(['version','attached','mounted','timeout_ms','dropped_events','dropped_rx'],e.status())))
  elif a.cmd=='load':e.load(json.loads(pathlib.Path(a.profile).read_text()));print(e.status())
  elif a.cmd=='press':
   end=time.monotonic()+a.seconds
   try:
    while time.monotonic()<end:e.report(a.interface,ps3_report(a.button.split(',')));time.sleep(.01)
   finally:e.report(a.interface,ps3_report())
  elif a.cmd=='detach':e.command(2)
  elif a.cmd=='boot':
   try:e.command(10,b'BOOT')
   except TimeoutError:pass
  else:
   while True:
    r=e.read(1)
    if r:print(time.time(),r[0],r[1],r[2].hex(),flush=True)
 finally:e.close()
if __name__=='__main__':main()
