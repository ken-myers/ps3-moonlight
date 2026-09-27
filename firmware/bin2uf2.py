import pathlib,struct,sys
p=pathlib.Path(sys.argv[1]);data=p.read_bytes();blocks=(len(data)+255)//256
with p.with_suffix('.uf2').open('wb') as f:
 for i in range(blocks):
  payload=data[i*256:i*256+256].ljust(256,b'\0')
  f.write(struct.pack('<8I',0x0a324655,0x9e5d5157,0x2000,0x10000000+i*256,256,i,blocks,0xe48bff59)+payload+bytes(220)+struct.pack('<I',0x0ab16f30))
