# -*- coding: utf-8 -*-
"""Render docs/RASPBOT-V2_测试方案_v1.0.md to a Word document.

The markdown subset used by the test plan is small and fixed:
headings, paragraphs, tables, fenced code, bullets, numbered items,
blockquotes, horizontal rules and HTML comments (ignored).

Layout follows the house style of the design documents: A4, Chinese body
font (宋体) with 黑体 headings, 9 pt tables with a shaded repeating header,
Consolas code blocks, footer with page numbers.
"""
from __future__ import annotations

import re
from pathlib import Path

from docx import Document
from docx.enum.section import WD_SECTION
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.enum.text import WD_ALIGN_PARAGRAPH, WD_BREAK
from docx.oxml import OxmlElement
from docx.oxml.ns import qn
from docx.shared import Cm, Pt, RGBColor

SRC = Path(r"E:\code\raspberrypi\docs\RASPBOT-V2_测试方案_v1.0.md")
DST = Path(r"E:\code\raspberrypi\docs\RASPBOT-V2_测试方案_v1.0.docx")

BODY_EA = "宋体"
BODY_LATIN = "Times New Roman"
HEAD_EA = "黑体"
HEAD_LATIN = "Arial"
MONO = "Consolas"

USABLE_CM = 17.0
CELL_PT = 9.0
CODE_PT = 8.5


# --------------------------------------------------------------- xml helpers
def set_run_font(run, latin: str, ea: str, size: float | None = None,
                 bold: bool | None = None, color: str | None = None) -> None:
    run.font.name = latin
    rpr = run._element.get_or_add_rPr()
    rfonts = rpr.find(qn("w:rFonts"))
    if rfonts is None:
        rfonts = OxmlElement("w:rFonts")
        rpr.append(rfonts)
    rfonts.set(qn("w:ascii"), latin)
    rfonts.set(qn("w:hAnsi"), latin)
    rfonts.set(qn("w:eastAsia"), ea)
    if size is not None:
        run.font.size = Pt(size)
    if bold is not None:
        run.font.bold = bold
    if color is not None:
        run.font.color.rgb = RGBColor.from_string(color)


def shade(element_pr, fill: str) -> None:
    shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear")
    shd.set(qn("w:color"), "auto")
    shd.set(qn("w:fill"), fill)
    element_pr.append(shd)


def shade_paragraph(paragraph, fill: str) -> None:
    shade(paragraph._p.get_or_add_pPr(), fill)


def shade_cell(cell, fill: str) -> None:
    shade(cell._tc.get_or_add_tcPr(), fill)


def keep_with_next(paragraph) -> None:
    ppr = paragraph._p.get_or_add_pPr()
    el = OxmlElement("w:keepNext")
    ppr.append(el)


def repeat_header(row) -> None:
    trpr = row._tr.get_or_add_trPr()
    el = OxmlElement("w:tblHeader")
    el.set(qn("w:val"), "true")
    trpr.append(el)


def cant_split(row) -> None:
    trpr = row._tr.get_or_add_trPr()
    el = OxmlElement("w:cantSplit")
    el.set(qn("w:val"), "true")
    trpr.append(el)


def add_field(paragraph, instr: str, size: float = 9.0, color: str = "595959") -> None:
    run = paragraph.add_run()
    fld_begin = OxmlElement("w:fldChar")
    fld_begin.set(qn("w:fldCharType"), "begin")
    instr_el = OxmlElement("w:instrText")
    instr_el.set(qn("xml:space"), "preserve")
    instr_el.text = f" {instr} "
    fld_sep = OxmlElement("w:fldChar")
    fld_sep.set(qn("w:fldCharType"), "separate")
    fld_end = OxmlElement("w:fldChar")
    fld_end.set(qn("w:fldCharType"), "end")
    run._r.append(fld_begin)
    run._r.append(instr_el)
    run._r.append(fld_sep)
    run._r.append(fld_end)
    set_run_font(run, BODY_LATIN, BODY_EA, size, color=color)


# ------------------------------------------------------------ inline parsing
TOKEN_RE = re.compile(r"(\*\*|`)")


def display_width(text: str) -> int:
    """Rough width in half-width units (CJK counts double)."""
    return sum(2 if ord(ch) > 0x2E7F else 1 for ch in text)


def add_inline(paragraph, text: str, size: float, base_bold: bool = False,
               color: str | None = None) -> None:
    """Render inline markdown, tolerating code nested inside bold spans."""
    bold = base_bold
    mono = False
    for token in TOKEN_RE.split(text):
        if token == "**":
            bold = not bold
            continue
        if token == "`":
            mono = not mono
            continue
        if not token:
            continue
        run = paragraph.add_run(token)
        if mono:
            set_run_font(run, MONO, MONO, max(size - 0.5, 7.0), bold=bold,
                         color="1F3864")
        else:
            set_run_font(run, BODY_LATIN, BODY_EA, size, bold=bold, color=color)


# ----------------------------------------------------------------- md parser
def parse_blocks(lines: list[str]):
    i, n = 0, len(lines)
    while i < n:
        line = lines[i].rstrip()
        stripped = line.strip()

        if not stripped or stripped.startswith("<!--"):
            i += 1
            continue

        if stripped == "---":
            i += 1
            continue

        m = re.match(r"^(#{1,4})\s+(.*)$", stripped)
        if m:
            yield ("heading", len(m.group(1)), m.group(2).strip())
            i += 1
            continue

        if stripped.startswith("```"):
            i += 1
            code: list[str] = []
            while i < n and not lines[i].strip().startswith("```"):
                code.append(lines[i].rstrip("\n"))
                i += 1
            i += 1
            yield ("code", code)
            continue

        if stripped.startswith("|"):
            rows: list[list[str]] = []
            while i < n and lines[i].strip().startswith("|"):
                raw = lines[i].strip()
                cells = [c.strip() for c in raw.strip("|").split("|")]
                if not all(re.fullmatch(r":?-{2,}:?", c) for c in cells):
                    rows.append(cells)
                i += 1
            if rows:
                yield ("table", rows)
            continue

        if stripped.startswith("> "):
            buf: list[str] = []
            while i < n and lines[i].strip().startswith("> "):
                buf.append(lines[i].strip()[2:].strip())
                i += 1
            yield ("quote", " ".join(buf))
            continue

        m = re.match(r"^(\s*)-\s+(.*)$", line)
        if m:
            yield ("bullet", len(m.group(1)) // 2, m.group(2).strip())
            i += 1
            continue

        m = re.match(r"^(\s*)(\d+)\.\s+(.*)$", line)
        if m:
            yield ("number", m.group(2), m.group(3).strip())
            i += 1
            continue

        buf = [stripped]
        i += 1
        while i < n:
            nxt = lines[i].strip()
            if (not nxt or nxt.startswith(("#", "|", "```", "> ", "- ", "<!--"))
                    or nxt == "---" or re.match(r"^\d+\.\s", nxt)):
                break
            buf.append(nxt)
            i += 1
        yield ("paragraph", " ".join(buf))


# ------------------------------------------------------------------ renderer
def setup_document() -> Document:
    doc = Document()
    sec = doc.sections[0]
    sec.page_width = Cm(21.0)
    sec.page_height = Cm(29.7)
    sec.top_margin = Cm(2.3)
    sec.bottom_margin = Cm(2.1)
    sec.left_margin = Cm(2.0)
    sec.right_margin = Cm(2.0)
    sec.header_distance = Cm(1.3)
    sec.footer_distance = Cm(1.2)

    normal = doc.styles["Normal"]
    normal.font.name = BODY_LATIN
    normal.font.size = Pt(10.5)
    normal.element.rPr.rFonts.set(qn("w:eastAsia"), BODY_EA)
    pf = normal.paragraph_format
    pf.line_spacing = 1.3
    pf.space_after = Pt(4)
    pf.space_before = Pt(0)

    # header
    hdr = sec.header.paragraphs[0]
    hdr.alignment = WD_ALIGN_PARAGRAPH.RIGHT
    run = hdr.add_run("RASPBOT-V2 智能驾驶小车测试方案 ｜ RB2-STP-001 ｜ v1.0")
    set_run_font(run, BODY_LATIN, BODY_EA, 8.5, color="7F7F7F")

    # footer with page fields
    ftr = sec.footer.paragraphs[0]
    ftr.alignment = WD_ALIGN_PARAGRAPH.CENTER
    r1 = ftr.add_run("第 ")
    set_run_font(r1, BODY_LATIN, BODY_EA, 9, color="595959")
    add_field(ftr, "PAGE")
    r2 = ftr.add_run(" 页 / 共 ")
    set_run_font(r2, BODY_LATIN, BODY_EA, 9, color="595959")
    add_field(ftr, "NUMPAGES")
    r3 = ftr.add_run(" 页")
    set_run_font(r3, BODY_LATIN, BODY_EA, 9, color="595959")
    return doc


def add_heading(doc: Document, level: int, text: str, first: bool = False):
    if level == 1:
        p = doc.add_paragraph()
        p.alignment = WD_ALIGN_PARAGRAPH.CENTER
        p.paragraph_format.space_before = Pt(60)
        p.paragraph_format.space_after = Pt(10)
        add_inline(p, text, 21.0, base_bold=True)
        for run in p.runs:
            if run.font.name != MONO:
                set_run_font(run, HEAD_LATIN, HEAD_EA, 21.0, bold=True, color="1F3864")
        return p

    if level == 2:
        p = doc.add_paragraph()
        p.paragraph_format.space_before = Pt(10)
        p.paragraph_format.space_after = Pt(8)
        add_inline(p, text, 15.0, base_bold=True)
        for run in p.runs:
            if run.font.name != MONO:
                set_run_font(run, HEAD_LATIN, HEAD_EA, 15.0, bold=True, color="1F3864")
        ppr = p._p.get_or_add_pPr()
        bdr = OxmlElement("w:pBdr")
        bottom = OxmlElement("w:bottom")
        bottom.set(qn("w:val"), "single")
        bottom.set(qn("w:sz"), "8")
        bottom.set(qn("w:space"), "2")
        bottom.set(qn("w:color"), "2E5496")
        bdr.append(bottom)
        ppr.append(bdr)
        keep_with_next(p)
        return p

    size = 12.5 if level == 3 else 11.0
    p = doc.add_paragraph()
    p.paragraph_format.space_before = Pt(8 if level == 3 else 6)
    p.paragraph_format.space_after = Pt(4)
    add_inline(p, text, size, base_bold=True)
    for run in p.runs:
        if run.font.name != MONO:
            set_run_font(run, HEAD_LATIN, HEAD_EA, size, bold=True, color="1F3864")
    keep_with_next(p)
    return p


def add_paragraph(doc: Document, text: str):
    p = doc.add_paragraph()
    p.paragraph_format.first_line_indent = Cm(0.0)
    add_inline(p, text, 10.5)
    p.alignment = WD_ALIGN_PARAGRAPH.JUSTIFY
    return p


def add_bullet(doc: Document, level: int, text: str):
    style = "List Bullet" if level == 0 else "List Bullet 2"
    try:
        p = doc.add_paragraph(style=style)
    except KeyError:
        p = doc.add_paragraph()
        text = ("· " if level == 0 else "– ") + text
    p.paragraph_format.space_after = Pt(2)
    p.paragraph_format.line_spacing = 1.25
    if level > 0:
        p.paragraph_format.left_indent = Cm(0.95)
    add_inline(p, text, 10.5)
    return p


def add_number(doc: Document, num: str, text: str):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.75)
    p.paragraph_format.first_line_indent = Cm(-0.75)
    p.paragraph_format.space_after = Pt(2)
    run = p.add_run(f"{num}. ")
    set_run_font(run, BODY_LATIN, BODY_EA, 10.5, bold=True)
    add_inline(p, text, 10.5)
    return p


def add_quote(doc: Document, text: str):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.6)
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(6)
    shade_paragraph(p, "F4F6FA")
    add_inline(p, text, 10.0, color="333333")
    return p


def add_code(doc: Document, lines: list[str]):
    p = doc.add_paragraph()
    p.paragraph_format.left_indent = Cm(0.3)
    p.paragraph_format.space_before = Pt(4)
    p.paragraph_format.space_after = Pt(8)
    p.paragraph_format.line_spacing = 1.05
    shade_paragraph(p, "F5F5F5")
    for idx, line in enumerate(lines):
        run = p.add_run(line)
        set_run_font(run, MONO, MONO, CODE_PT, color="1A1A1A")
        if idx != len(lines) - 1:
            run.add_break()
    return p


def add_table(doc: Document, rows: list[list[str]]):
    ncols = max(len(r) for r in rows)
    rows = [r + [""] * (ncols - len(r)) for r in rows]

    table = doc.add_table(rows=len(rows), cols=ncols)
    table.style = "Table Grid"
    table.alignment = WD_TABLE_ALIGNMENT.CENTER
    table.autofit = False

    widths = [max(display_width(r[c]) for r in rows) for c in range(ncols)]
    widths = [min(max(w, 5), 60) for w in widths]
    total = sum(widths)
    raw = [USABLE_CM * w / total for w in widths]
    raw = [max(1.55, x) for x in raw]
    scale = USABLE_CM / sum(raw)
    widths_cm = [x * scale for x in raw]

    # keep the identifier column wide enough that IDs never hyphen-break
    if rows[0][0].strip() in ("ID", "编号") and len(widths_cm) > 1:
        first = max(widths_cm[0], 2.05)
        rest_total = sum(widths_cm[1:])
        if rest_total > 0:
            factor = (USABLE_CM - first) / rest_total
            widths_cm = [first] + [w * factor for w in widths_cm[1:]]

    # fixed layout + explicit grid so every renderer honours the widths
    tbl_pr = table._tbl.tblPr
    layout = OxmlElement("w:tblLayout")
    layout.set(qn("w:type"), "fixed")
    tbl_pr.append(layout)
    for idx, width in enumerate(widths_cm):
        table.columns[idx].width = Cm(width)

    for r_idx, row in enumerate(rows):
        tr = table.rows[r_idx]
        cant_split(tr)
        if r_idx == 0:
            repeat_header(tr)
        for c_idx, text in enumerate(row):
            cell = tr.cells[c_idx]
            cell.width = Cm(widths_cm[c_idx])
            para = cell.paragraphs[0]
            para.paragraph_format.space_before = Pt(1)
            para.paragraph_format.space_after = Pt(1)
            para.paragraph_format.line_spacing = 1.15
            plain = text.replace("<br>", " ")
            if r_idx == 0:
                para.alignment = WD_ALIGN_PARAGRAPH.CENTER
                add_inline(para, plain, CELL_PT, base_bold=True)
                for run in para.runs:
                    set_run_font(run, BODY_LATIN, BODY_EA, CELL_PT, bold=True,
                                 color="1F3864")
                shade_cell(cell, "E8EEF7")
            else:
                add_inline(para, plain, CELL_PT)
                for run in para.runs:
                    if run.font.size is None:
                        set_run_font(run, BODY_LATIN, BODY_EA, CELL_PT)
                    elif run.font.size > Pt(CELL_PT):
                        run.font.size = Pt(CELL_PT)
    # small spacing after table
    tail = doc.add_paragraph()
    tail.paragraph_format.space_before = Pt(0)
    tail.paragraph_format.space_after = Pt(2)
    tail.paragraph_format.line_spacing = 1.0
    for run in tail.runs:
        run.font.size = Pt(4)
    return table


def main() -> None:
    lines = SRC.read_text(encoding="utf-8").splitlines()
    doc = setup_document()
    first_heading = True
    chapter_re = re.compile(r"^\d+\s")

    for block in parse_blocks(lines):
        kind = block[0]
        if kind == "heading":
            level, text = block[1], block[2]
            if level == 2 and chapter_re.match(text):
                doc.add_page_break()
            elif level == 3 and text.startswith("文档结构"):
                # cover page stays clean: info + revision table only
                doc.add_page_break()
            add_heading(doc, level, text, first=first_heading and level == 1)
            first_heading = False
        elif kind == "paragraph":
            add_paragraph(doc, block[1])
        elif kind == "bullet":
            add_bullet(doc, block[1], block[2])
        elif kind == "number":
            add_number(doc, block[1], block[2])
        elif kind == "quote":
            add_quote(doc, block[1])
        elif kind == "code":
            add_code(doc, block[1])
        elif kind == "table":
            add_table(doc, block[1])

    # document properties
    doc.core_properties.title = "RASPBOT-V2 智能驾驶小车 测试方案"
    doc.core_properties.subject = "RB2-STP-001 v1.0"
    doc.core_properties.author = "项目组"
    doc.core_properties.comments = "软件测试 / 系统测试 / 实机测试 三级测试方案"
    doc.save(str(DST))
    print(f"saved {DST} ({DST.stat().st_size} bytes)")


if __name__ == "__main__":
    main()
