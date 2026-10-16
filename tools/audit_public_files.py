#!/usr/bin/env python3
"""Read-only candidate-tree privacy flags and inventories. No license verdict."""
import argparse,hashlib,ipaddress,json,re
from pathlib import Path


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output',type=Path)
    args=p.parse_args()
    root=Path(__file__).resolve().parent.parent
    files=[];flags=[]
    for f in sorted(root.rglob('*')):
        if not f.is_file() or '.git' in f.parts or '__pycache__' in f.parts:continue
        rel=str(f.relative_to(root))
        if args.output and f.resolve()==args.output.resolve():continue
        files.append(dict(path=rel,bytes=f.stat().st_size,sha256=hashlib.sha256(f.read_bytes()).hexdigest()))
        if f.name.endswith('_calibration.json') or f.suffix.lower() in ('.pem','.key','.p12','.pt','.bin','.exe'):
            flags.append(dict(path=rel,reason='private calibration/checkpoint/key/vendor binary candidate'))
        if f.suffix.lower() not in ('.py','.md','.json','.yaml','.yml','.toml','.sh','.xml','.csv'):continue
        try:lines=f.read_text().splitlines()
        except UnicodeDecodeError:continue
        for i,line in enumerate(lines,1):
            for token in re.findall(r'(?<![\d.])(?:\d{1,3}\.){3}\d{1,3}(?![\d.])',line):
                try:address=ipaddress.ip_address(token)
                except ValueError:continue
                if address.is_private and not address.is_loopback:flags.append(dict(path=rel,line=i,reason='private IP literal; inspect context'))
            if re.search(r'-----BEGIN (?:RSA |OPENSSH |EC )?PRIVATE KEY-----|gh[pousr]_[A-Za-z0-9]{30,}|AKIA[0-9A-Z]{16}',line):
                flags.append(dict(path=rel,line=i,reason='credential-like pattern'))
            if re.search(r'/home/(?:ubuntu|antoine|zj)/',line):flags.append(dict(path=rel,line=i,reason='personal machine path'))
    report=dict(file_count=len(files),flags=flags,files=files,license_status='manual file-by-file review pending',hardware_opened=False,inference_executed=False)
    text=json.dumps(report,ensure_ascii=False,indent=2)+'\n'
    if args.output:args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(text)
    else:print(text)
    return 1 if flags else 0


if __name__=='__main__':raise SystemExit(main())
