"""Update raw GDScript overrides in an unencrypted Godot PCK v4; keep imported assets intact."""
import hashlib,struct,sys
from pathlib import Path

def repack(original,source,output):
 raw=bytearray(Path(original).read_bytes())
 magic,version,major,minor,patch,flags=struct.unpack_from('<6I',raw)
 if (magic,version,major,flags)!=(0x43504447,4,4,2):raise ValueError('Expected unencrypted Godot 4 PCK v4 with relative file offsets')
 base,directory=struct.unpack_from('<QQ',raw,24); cursor=directory
 count=struct.unpack_from('<I',raw,cursor)[0];cursor+=4;entries={}
 for _ in range(count):
  length=struct.unpack_from('<I',raw,cursor)[0];cursor+=4
  name=bytes(raw[cursor:cursor+length]).rstrip(b'\0').decode();cursor+=length
  offset,size,digest,entry_flags=struct.unpack_from('<QQ16sI',raw,cursor);cursor+=36
  if entry_flags!=0 or base+offset+size>directory:raise ValueError('Unsupported PCK entry')
  if hashlib.md5(raw[base+offset:base+offset+size]).digest()!=digest:raise ValueError('Checksum mismatch: '+name)
  entries[name]=(offset,size,digest,entry_flags)
 body=raw[:base]
 replacements={'scripts/main.gd.remap':b'[remap]\n\npath="res://scripts/main_runtime.gd"\n','scripts/main_runtime.gd':Path(source).read_bytes()}
 for script_name in ["minimap"]:
  script=Path(source).with_name(script_name+".gd")
  if script.exists():
   replacements["scripts/"+script_name+".gd.remap"]=('[remap]\n\npath="res://scripts/'+script_name+'_runtime.gd"\n').encode()
   replacements["scripts/"+script_name+"_runtime.gd"]=script.read_bytes()
 cache_name='.godot/global_script_class_cache.cfg'
 if 'scripts/minimap_runtime.gd' in replacements:
  offset,size,_,_=entries[cache_name]
  cache=bytes(raw[base+offset:base+offset+size]).replace(b'res://scripts/minimap.gd',b'res://scripts/minimap_runtime.gd')
  replacements[cache_name]=cache
 payloads={name:bytes(raw[base+off:base+off+size]) for name,(off,size,_,_) in entries.items()}
 payloads.update(replacements)
 entries={}
 for name,data in sorted(payloads.items()):
  body.extend(b'\0'*(-len(body)%16));entries[name]=(len(body)-base,len(data),hashlib.md5(data).digest(),0);body.extend(data)
 body.extend(b'\0'*(-len(body)%16));struct.pack_into('<Q',body,32,len(body));body.extend(struct.pack('<I',len(entries)))
 for name,(offset,size,digest,entry_flags) in sorted(entries.items()):
  encoded=name.encode();encoded+=b'\0'*(-len(encoded)%4)
  body.extend(struct.pack('<I',len(encoded))+encoded+struct.pack('<QQ16sI',offset,size,digest,entry_flags))
 Path(output).write_bytes(body)
 print('Verified and preserved',len(entries)-len(replacements),'asset entries; wrote',output)
if __name__=='__main__':
 repack(*sys.argv[1:])
