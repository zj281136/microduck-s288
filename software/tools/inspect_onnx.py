#!/usr/bin/env python3
"""Inspect a policy file without ONNX Runtime, simulation, serial or inference."""
import argparse
import hashlib
import json
from pathlib import Path
import onnx


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model',type=Path,required=True)
    p.add_argument('--manifest',type=Path)
    args=p.parse_args()
    model=onnx.load(args.model,load_external_data=False)
    def tensor(x):
        t=x.type.tensor_type
        return dict(name=x.name,dtype=onnx.TensorProto.DataType.Name(t.elem_type),shape=[d.dim_value if d.HasField('dim_value') else d.dim_param for d in t.shape.dim])
    report=dict(sha256=hashlib.sha256(args.model.read_bytes()).hexdigest(),opsets=[dict(domain=x.domain,version=x.version) for x in model.opset_import],inputs=[tensor(x) for x in model.graph.input],outputs=[tensor(x) for x in model.graph.output],metadata={p.key:p.value for p in model.metadata_props},inference_executed=False,hardware_accessed=False)
    manifest=args.manifest or args.model.with_suffix('.manifest.json')
    if manifest.exists():
        m=json.loads(manifest.read_text())
        if report['sha256']!=m.get('onnx_sha256'):raise ValueError('manifest/model SHA256 mismatch')
        if report['inputs'][0]['shape']!=m.get('input_shape') or report['outputs'][0]['shape']!=m.get('output_shape'):raise ValueError('manifest/model shape mismatch')
        report['file_manifest_consistent']=True
    print(json.dumps(report,indent=2,ensure_ascii=False))


if __name__=='__main__':main()
