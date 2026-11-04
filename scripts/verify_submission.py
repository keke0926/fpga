from pathlib import Path
import hashlib,json,xml.etree.ElementTree as ET
R=Path(__file__).resolve().parents[1]
manifest=json.loads((R/'evidence/SUBMISSION_MANIFEST.json').read_text(encoding='utf-8'))
sha=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
for name,digest in manifest['source_sha256'].items():
    p=R/'project'/name
    assert p.is_file() and sha(p)==digest,'Source mismatch: '+name
b=manifest['bitstream'];assert sha(R/b['path'])==b['sha256'],'Bitstream mismatch'
ns={'e':'http://www.efinixinc.com/enf_proj'};root=ET.parse(R/'project/Ti60_Demo.xml').getroot()
for tag in ('design_file','sdc_file'):
    for node in root.findall('.//e:'+tag,ns):assert (R/'project'/node.attrib['name']).is_file(),node.attrib['name']
print('PASS:',len(manifest['source_sha256']),'source files; project references; full-feature V16 bitstream')
