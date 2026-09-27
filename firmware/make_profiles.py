import re,json,pathlib,struct
root=pathlib.Path(__file__).parent
src=(root/'vendor/tusb_gamepad/src/descriptors/PS3Descriptors.h').read_text()
src=re.sub(r'//[^\n]*','',src)
def arr(name):
 s=re.search(r'\b'+name+r'\[\]\s*=\s*\{(.*?)\}',src,re.S)[1]
 return bytes(int(x,16) for x in re.findall(r'0x([0-9A-Fa-f]+)',s))
def string(s):
 b=s.encode('utf-16le');return bytes([len(b)+2,3])+b
for count in (1,2,4):
 report=arr('ps3_report_descriptor');config=bytearray([9,2,0,0,count,1,0,0x80,125])
 for i in range(count):
  config+=bytes([9,4,i,0,2,3,0,0,0,9,0x21,0x11,1,0,1,0x22])+struct.pack('<H',len(report))
  config+=bytes([7,5,2*i+2,3,64,0,1,7,5,0x81+2*i,3,64,0,1])
 config[2:4]=struct.pack('<H',len(config))
 neutral=bytearray(49);neutral[0]=1;neutral[6:10]=bytes([128]*4);neutral[30]=2;neutral[31]=5;neutral[32]=0x10
 rules=[]
 for i in range(count):
  targets={}
  for rid in ('01','f2','f5','ef','f7','f8'):
   match=struct.pack('<BBHHH',0xa1,1,0x300+int(rid,16),i,0)
   targets[rid]=len(rules)
   rules.append(dict(match=match.hex(),mask='ffffffffffff0000',action=1,data=arr('report_'+rid).hex()))
  for rid,offset,dst,n in (('f5',2,2,6),('ef',6,7,1)):
   match=struct.pack('<BBHHH',0x21,9,0x300+int(rid,16),i,0)
   rules.append(dict(match=match.hex(),mask='ffffffffffff0000',action=4,data=bytes([targets[rid],offset,dst,n]).hex()))
 profile=dict(name=f'DualShock3-{count}',source='https://github.com/wiredopposite/tusb_gamepad',device=arr('ps3_device_descriptor').hex(),configuration=config.hex(),reports=[report.hex()]*count,strings=['04030904',string('Sony').hex(),string('PLAYSTATION(R)3 Controller').hex()],rules=rules,neutral=[neutral.hex()]*count,timeout_ms=1000)
 (root/f'ps3-{count}.json').write_text(json.dumps(profile,indent=2)+'\n')
