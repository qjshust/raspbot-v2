import sys
from docx import Document
from docx.table import Table
from docx.text.paragraph import Paragraph

def iter_block_items(parent):
    from docx.oxml.ns import qn
    parent_elm = parent.element.body
    for child in parent_elm.iterchildren():
        if child.tag == qn('w:p'):
            yield Paragraph(child, parent)
        elif child.tag == qn('w:tbl'):
            yield Table(child, parent)

path = sys.argv[1]
doc = Document(path)
out = []
for block in iter_block_items(doc):
    if isinstance(block, Paragraph):
        t = block.text.strip()
        if t:
            style = block.style.name if block.style else ''
            out.append(f"[{style}] {t}" if style and style not in ('Normal',) else t)
    else:
        for row in block.rows:
            cells = [c.text.strip().replace('\n', ' / ') for c in row.cells]
            out.append(' | '.join(cells))
        out.append('')
print('\n'.join(out))
