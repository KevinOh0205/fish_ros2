#!/usr/bin/env python3
# operation.md -> operation.pdf 변환기.
#
# 이 Pi 에는 pandoc 도 pip 도 브라우저 CLI 인쇄도 없고 libreoffice 만 있다.
# HTML 을 거치면 libreoffice 가 표 열 폭·nowrap 을 무시해 레이아웃이 깨지므로
# (btn1 이 b/t/n/1 로 세로 쪼개졌다), **flat ODT(.fodt) 를 직접 생성**한다 —
# ODF 의 열 폭은 네이티브 속성이라 확실히 적용된다.
#
# 범용 마크다운 파서가 아니라 이 저장소 문서가 실제로 쓰는 문법만 처리한다:
#   제목(#~####) · 표 · 펜스 코드블록 · 목록(-, 숫자, 체크박스) · 인용(>) ·
#   굵게(**) · 인라인 코드(`) · 수평선(---)
#
# 사용:  python3 docs/make_pdf.py [입력.md] [출력.pdf]
#        (기본: docs/operation.md -> docs/operation.pdf)

import re
import subprocess
import sys
import tempfile
from pathlib import Path

CONTENT_CM = 17.0          # A4 21cm − 좌우 여백 2cm×2


def esc(s: str) -> str:
    return s.replace("&", "&amp;").replace("<", "&lt;").replace(">", "&gt;")


def spaces(s: str) -> str:
    # ODF 는 연속 공백을 합친다 — 코드 줄의 들여쓰기를 text:s 로 보존
    def rep(m):
        n = len(m.group(0))
        return f' <text:s text:c="{n-1}"/>' if n > 1 else " "
    return re.sub(r"  +", rep, s)


def inline(s: str) -> str:
    """**굵게** 와 `코드` 를 text:span 으로."""
    s = esc(s).replace("\\|", "|")
    s = re.sub(r"\*\*(.+?)\*\*", r'<text:span text:style-name="TB">\1</text:span>', s)
    s = re.sub(r"`([^`]+)`", r'<text:span text:style-name="TC">\1</text:span>', s)
    return s


def para(style: str, content: str) -> str:
    return f'<text:p text:style-name="{style}">{content}</text:p>'


def vis_len(t: str) -> int:
    # 한글은 라틴의 두 배 폭으로 렌더된다. 마크다운 장식은 폭이 아니다.
    t = re.sub(r"[*`]", "", t)
    return sum(2 if ord(ch) > 0x1100 else 1 for ch in t)


def split_cells(line: str):
    return [c.strip() for c in re.split(r"(?<!\\)\|", line.strip().strip("|"))]


def make_table(rows, tid):
    """(자동 스타일 XML, 본문 XML). 열 폭은 내용 길이에 비례해 cm 로 확정한다."""
    data = [r for r in rows if not re.match(r"^\s*\|[\s:|-]+\|\s*$", r)]
    grid = [split_cells(r) for r in data]
    ncol = max(len(g) for g in grid)
    maxlen = [max((vis_len(g[c]) for g in grid if c < len(g)), default=1) or 1
              for c in range(ncol)]
    capped = [min(m, 60) for m in maxlen]
    total = sum(capped)
    width_cm = [max(1.6, c * CONTENT_CM / total) for c in capped]
    scale = CONTENT_CM / sum(width_cm)
    width_cm = [w * scale for w in width_cm]

    styles = [f'<style:style style:name="Tab{tid}" style:family="table">'
              f'<style:table-properties style:width="{CONTENT_CM}cm" table:align="left"/>'
              f'</style:style>']
    for j, w in enumerate(width_cm):
        styles.append(f'<style:style style:name="Tab{tid}.C{j}" style:family="table-column">'
                      f'<style:table-column-properties style:column-width="{w:.3f}cm"/>'
                      f'</style:style>')

    body = [f'<table:table table:name="T{tid}" table:style-name="Tab{tid}">']
    for j in range(ncol):
        body.append(f'<table:table-column table:style-name="Tab{tid}.C{j}"/>')
    for ri, g in enumerate(grid):
        cellstyle, parastyle = ("CellH", "PH") if ri == 0 else ("Cell", "PC")
        body.append("<table:table-row>")
        for j in range(ncol):
            c = g[j] if j < len(g) else ""
            body.append(f'<table:table-cell table:style-name="{cellstyle}" '
                        f'office:value-type="string">{para(parastyle, inline(c))}'
                        f"</table:table-cell>")
        body.append("</table:table-row>")
    body.append("</table:table>")
    return "\n".join(styles), "\n".join(body)


def convert(md: str):
    out, tstyles = [], []
    lines = md.splitlines()
    i, in_code, tid = 0, False, 0
    while i < len(lines):
        ln = lines[i]

        if ln.strip().startswith("```"):
            in_code = not in_code
            i += 1
            continue
        if in_code:
            out.append(para("Code", spaces(esc(ln)) or "&#160;"))
            i += 1
            continue

        m = re.match(r"^(#{1,4})\s+(.*)", ln)
        if m:
            out.append(para(f"H{len(m.group(1))}", inline(m.group(2))))
        elif re.match(r"^-{3,}\s*$", ln):
            out.append(para("HR", ""))
        elif ln.startswith(">"):
            body = re.sub(r"^#+\s*", "", ln.lstrip(">").strip())
            out.append(para("Quote", inline(body) if body else ""))
        elif ln.strip().startswith("|"):
            rows = []
            while i < len(lines) and lines[i].strip().startswith("|"):
                rows.append(lines[i])
                i += 1
            i -= 1
            tid += 1
            ts, tb = make_table(rows, tid)
            tstyles.append(ts)
            out.append(tb)
        elif re.match(r"^\s*-\s+", ln):
            item = re.sub(r"^\s*-\s+", "", ln)
            item = item.replace("[ ]", "☐").replace("[x]", "☑")
            out.append(para("LI", "• " + inline(item)))
        elif re.match(r"^\s*\d+\.\s+", ln):
            out.append(para("LI", inline(ln.strip())))
        elif ln.strip() == "":
            pass                                    # 문단 간격은 스타일 여백이 담당
        else:
            out.append(para("P", inline(ln)))
        i += 1
    return "\n".join(tstyles), "\n".join(out)


FODT = """<?xml version="1.0" encoding="UTF-8"?>
<office:document
 xmlns:office="urn:oasis:names:tc:opendocument:xmlns:office:1.0"
 xmlns:style="urn:oasis:names:tc:opendocument:xmlns:style:1.0"
 xmlns:text="urn:oasis:names:tc:opendocument:xmlns:text:1.0"
 xmlns:table="urn:oasis:names:tc:opendocument:xmlns:table:1.0"
 xmlns:fo="urn:oasis:names:tc:opendocument:xmlns:xsl-fo-compatible:1.0"
 xmlns:svg="urn:oasis:names:tc:opendocument:xmlns:svg-compatible:1.0"
 office:version="1.2" office:mimetype="application/vnd.oasis.opendocument.text">
<office:font-face-decls>
 <style:font-face style:name="KR" svg:font-family="&apos;Noto Sans CJK KR&apos;"/>
 <style:font-face style:name="MONO" svg:font-family="&apos;Noto Sans Mono CJK KR&apos;"/>
</office:font-face-decls>
<office:styles>
 <style:default-style style:family="paragraph">
  <style:text-properties style:font-name="KR" fo:font-size="10.5pt" fo:language="ko" fo:country="KR" style:font-name-asian="KR" style:font-size-asian="10.5pt" style:language-asian="ko" style:country-asian="KR"/>
  <style:paragraph-properties fo:line-height="145%"/>
 </style:default-style>
</office:styles>
<office:automatic-styles>
 <style:page-layout style:name="PL">
  <style:page-layout-properties fo:page-width="21cm" fo:page-height="29.7cm"
   fo:margin-top="1.8cm" fo:margin-bottom="1.8cm" fo:margin-left="2cm" fo:margin-right="2cm"/>
 </style:page-layout>
 <style:style style:name="P" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.1cm" fo:margin-bottom="0.1cm"/>
 </style:style>
 <style:style style:name="H1" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.3cm" fo:margin-bottom="0.25cm"
   fo:border-bottom="1.5pt solid #333333" fo:padding-bottom="0.1cm" fo:keep-with-next="always"/>
  <style:text-properties fo:font-size="16pt" fo:font-weight="bold" style:font-size-asian="16pt" style:font-weight-asian="bold"/>
 </style:style>
 <style:style style:name="H2" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.5cm" fo:margin-bottom="0.2cm"
   fo:border-bottom="0.8pt solid #999999" fo:padding-bottom="0.06cm" fo:keep-with-next="always"/>
  <style:text-properties fo:font-size="13.5pt" fo:font-weight="bold" style:font-size-asian="13.5pt" style:font-weight-asian="bold"/>
 </style:style>
 <style:style style:name="H3" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.4cm" fo:margin-bottom="0.15cm" fo:keep-with-next="always"/>
  <style:text-properties fo:font-size="11.5pt" fo:font-weight="bold" style:font-size-asian="11.5pt" style:font-weight-asian="bold"/>
 </style:style>
 <style:style style:name="H4" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.3cm" fo:margin-bottom="0.1cm" fo:keep-with-next="always"/>
  <style:text-properties fo:font-size="10.5pt" fo:font-weight="bold" style:font-size-asian="10.5pt" style:font-weight-asian="bold"/>
 </style:style>
 <style:style style:name="Code" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0cm" fo:margin-bottom="0cm"
   fo:background-color="#f4f4f4" fo:padding-left="0.2cm" fo:line-height="130%"/>
  <style:text-properties style:font-name="MONO" fo:font-size="8pt" style:font-name-asian="MONO" style:font-size-asian="8pt"/>
 </style:style>
 <style:style style:name="Quote" style:family="paragraph">
  <style:paragraph-properties fo:margin-left="0.3cm" fo:padding-left="0.25cm"
   fo:border-left="2.5pt solid #999999" fo:margin-top="0.05cm" fo:margin-bottom="0.05cm"/>
  <style:text-properties fo:color="#333333" fo:font-size="9.5pt" style:font-size-asian="9.5pt"/>
 </style:style>
 <style:style style:name="LI" style:family="paragraph">
  <style:paragraph-properties fo:margin-left="0.5cm" fo:text-indent="-0.3cm"
   fo:margin-top="0.05cm" fo:margin-bottom="0.05cm"/>
 </style:style>
 <style:style style:name="HR" style:family="paragraph">
  <style:paragraph-properties fo:border-bottom="0.8pt solid #888888"
   fo:margin-top="0.2cm" fo:margin-bottom="0.3cm"/>
 </style:style>
 <style:style style:name="PC" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.03cm" fo:margin-bottom="0.03cm"/>
  <style:text-properties fo:font-size="9pt" style:font-size-asian="9pt"/>
 </style:style>
 <style:style style:name="PH" style:family="paragraph">
  <style:paragraph-properties fo:margin-top="0.03cm" fo:margin-bottom="0.03cm"/>
  <style:text-properties fo:font-size="9pt" fo:font-weight="bold" style:font-size-asian="9pt" style:font-weight-asian="bold"/>
 </style:style>
 <style:style style:name="TB" style:family="text">
  <style:text-properties fo:font-weight="bold" style:font-weight-asian="bold"/>
 </style:style>
 <style:style style:name="TC" style:family="text">
  <style:text-properties style:font-name="MONO" fo:font-size="9pt"
   fo:background-color="#efefef" style:font-name-asian="MONO" style:font-size-asian="9pt"/>
 </style:style>
 <style:style style:name="Cell" style:family="table-cell">
  <style:table-cell-properties fo:border="0.5pt solid #666666" fo:padding="0.09cm"/>
 </style:style>
 <style:style style:name="CellH" style:family="table-cell">
  <style:table-cell-properties fo:border="0.5pt solid #666666" fo:padding="0.09cm"
   fo:background-color="#e8e8e8"/>
 </style:style>
{TABLE_STYLES}
</office:automatic-styles>
<office:master-styles>
 <style:master-page style:name="Standard" style:page-layout-name="PL"/>
</office:master-styles>
<office:body><office:text>
{BODY}
</office:text></office:body>
</office:document>
"""


def main():
    src = Path(sys.argv[1] if len(sys.argv) > 1 else Path(__file__).parent / "operation.md")
    dst = Path(sys.argv[2] if len(sys.argv) > 2 else src.with_suffix(".pdf"))

    tstyles, body = convert(src.read_text(encoding="utf-8"))
    doc = FODT.replace("{TABLE_STYLES}", tstyles).replace("{BODY}", body)

    with tempfile.TemporaryDirectory() as td:
        f = Path(td) / (src.stem + ".fodt")
        f.write_text(doc, encoding="utf-8")
        subprocess.run(
            ["libreoffice", "--headless", "--convert-to", "pdf", "--outdir", td, str(f)],
            check=True, capture_output=True)
        pdf = Path(td) / (src.stem + ".pdf")

        # libreoffice 는 한글(CFF 계열 Noto CJK)을 구형 Type 1 서브셋으로 임베드하는데,
        # 이 형식을 못 읽는 뷰어가 많다 (GitHub 미리보기 등에서 글자가 깨진 실사례).
        # ghostscript 로 재증류하면 Type 1C 로 바뀌어 어디서나 열리고 용량도 절반 이하다.
        gs_out = Path(td) / (src.stem + "_gs.pdf")
        r = subprocess.run(
            ["gs", "-dNOPAUSE", "-dBATCH", "-dQUIET", "-sDEVICE=pdfwrite",
             "-dCompatibilityLevel=1.7", "-dPDFSETTINGS=/prepress",
             "-o", str(gs_out), str(pdf)], capture_output=True)
        dst.write_bytes((gs_out if r.returncode == 0 else pdf).read_bytes())
        if r.returncode != 0:
            print("경고: ghostscript 재증류 실패 — libreoffice 원본을 그대로 씁니다")
    print(f"{dst}  ({dst.stat().st_size/1024:.0f} KB)")


if __name__ == "__main__":
    main()
