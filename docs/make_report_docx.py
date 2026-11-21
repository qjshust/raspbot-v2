"""Generate 自测试报告 docx from the markdown report (simple, structured)."""
import re
import sys

from docx import Document
from docx.shared import Pt

src = r'E:\code\raspberrypi\docs\自测试报告_2026-11-21.md'
dst = r'E:\code\raspberrypi\docs\自测试报告_2026-11-21.docx'

doc = Document()
style = doc.styles['Normal']
style.font.name = 'Calibri'
style.font.size = Pt(10.5)

with open(src, encoding='utf-8') as f:
    lines = f.read().splitlines()

i = 0
while i < len(lines):
    line = lines[i].rstrip()
    if not line.strip():
        i += 1
        continue
    if line.startswith('# '):
        doc.add_heading(line[2:], level=0)
    elif line.startswith('## '):
        doc.add_heading(line[3:], level=1)
    elif line.startswith('### '):
        doc.add_heading(line[4:], level=2)
    elif line.startswith('|'):
        # collect table block
        block = []
        while i < len(lines) and lines[i].strip().startswith('|'):
            block.append(lines[i].strip())
            i += 1
        rows = []
        for r in block:
            cells = [c.strip() for c in r.strip('|').split('|')]
            rows.append(cells)
        if rows:
            ncols = max(len(r) for r in rows)
            t = doc.add_table(rows=len(rows), cols=ncols)
            t.style = 'Light Grid Accent 1'
            for ri, r in enumerate(rows):
                for ci in range(ncols):
                    t.cell(ri, ci).text = r[ci] if ci < len(r) else ''
        continue
    elif line.startswith('```'):
        i += 1
        while i < len(lines) and not lines[i].strip().startswith('```'):
            i += 1
    elif line.startswith('- '):
        doc.add_paragraph(line[2:], style='List Bullet')
    elif re.match(r'^\d+\. ', line):
        doc.add_paragraph(line, style='List Number')
    elif line.strip() == '---':
        pass
    else:
        doc.add_paragraph(line)
    i += 1

doc.save(dst)
print('saved', dst)
