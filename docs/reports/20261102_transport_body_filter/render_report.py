#!/usr/bin/env python3
"""Render this report's Markdown subset to a self-contained image HTML page.

Uses only the standard library. It handles headings, paragraphs, flat lists,
tables, code fences, links, images, bold, and inline code used by these reports.
Relative evidence links remain relative; figures are embedded for portability.
"""
from pathlib import Path
import base64
import html
import re

ROOT = Path(__file__).resolve().parent


def inline(text):
    stored = []

    def hold(markup):
        stored.append(markup)
        return f'\x00{len(stored)-1}\x00'

    text = re.sub(r'`([^`]+)`', lambda m: hold('<code>'+html.escape(m[1])+'</code>'), text)

    def image(m):
        p = (ROOT / m[2]).resolve()
        data = base64.b64encode(p.read_bytes()).decode('ascii')
        return hold(f'<figure><img src="data:image/png;base64,{data}" alt="{html.escape(m[1],quote=True)}">'
                    f'<figcaption>{html.escape(m[1])}</figcaption></figure>')

    text = re.sub(r'!\[([^\]]+)\]\(([^)]+)\)', image, text)
    text = re.sub(r'\[([^\]]+)\]\(([^)]+)\)', lambda m: hold(
        '<a href="'+html.escape(m[2],quote=True)+'">'+html.escape(m[1])+'</a>'), text)
    text = html.escape(text)
    text = re.sub(r'\*\*(.+?)\*\*',r'<strong>\1</strong>',text)
    return re.sub(r'\x00(\d+)\x00',lambda m:stored[int(m[1])],text)


def render(path):
    lines=path.read_text().splitlines(); blocks=[]; toc=[]; i=0; hid=0
    while i<len(lines):
        line=lines[i]
        if not line.strip(): i+=1; continue
        if line.startswith('```'):
            lang=line[3:].strip(); i+=1; code=[]
            while i<len(lines) and not lines[i].startswith('```'):
                code.append(lines[i]);i+=1
            blocks.append('<pre data-language="'+html.escape(lang,quote=True)+'"><code>'+html.escape('\n'.join(code))+'</code></pre>')
            i+=1;continue
        heading=re.match(r'^(#{1,6}) (.*)$',line)
        if heading:
            level=len(heading[1]);hid+=1;anchor='section-'+str(hid)
            blocks.append(f'<h{level} id="{anchor}">'+inline(heading[2])+f'</h{level}>')
            if level==2: toc.append((anchor,heading[2]))
            i+=1;continue
        if line.startswith('|'):
            rows=[]
            while i<len(lines) and lines[i].startswith('|'):
                row=[c.strip() for c in lines[i].strip().strip('|').split('|')]
                if not all(re.fullmatch(r':?-+:?',c) for c in row):rows.append(row)
                i+=1
            head='<thead><tr>'+''.join('<th>'+inline(c)+'</th>' for c in rows[0])+'</tr></thead>'
            body='<tbody>'+''.join('<tr>'+''.join('<td>'+inline(c)+'</td>' for c in row)+'</tr>' for row in rows[1:])+'</tbody>'
            blocks.append('<div class="table-wrap"><table>'+head+body+'</table></div>');continue
        if re.match(r'^(- |\d+\. )',line):
            ordered=bool(re.match(r'^\d+\. ',line));tag='ol' if ordered else 'ul';items=[]
            while i<len(lines) and re.match(r'^(- |\d+\. )',lines[i]):
                items.append('<li>'+inline(re.sub(r'^(- |\d+\. )','',lines[i]))+'</li>');i+=1
            blocks.append('<'+tag+'>'+''.join(items)+'</'+tag+'>');continue
        if line.startswith('!['):blocks.append(inline(line));i+=1;continue
        para=[line];i+=1
        while i<len(lines) and lines[i].strip() and not re.match(r'^(#|```|\||!\[|- |\d+\. )',lines[i]):
            para.append(lines[i]);i+=1
        blocks.append('<p>'+inline(' '.join(para))+'</p>')
    nav='<nav aria-label="목차"><b>목차</b><ul>'+''.join(f'<li><a href="#{a}">{html.escape(t)}</a></li>' for a,t in toc)+'</ul></nav>'
    title=lines[0].lstrip('# ')
    style='''
    :root{color-scheme:light}*{box-sizing:border-box}body{margin:0;background:#eef2f5;color:#1e2c37;font:16px/1.8 "Noto Sans CJK KR","Noto Sans KR",sans-serif}
    header{background:#19394d;color:white;padding:20px 32px}header a{color:#cceaff}.layout{display:grid;grid-template-columns:270px minmax(0,1120px);gap:25px;max-width:1470px;margin:28px auto;padding:0 20px}
    nav{align-self:start;position:sticky;top:20px;font-size:14px}nav ul{padding-left:19px}nav a{text-decoration:none;color:#204e70}main{padding:35px 45px;background:white;border-radius:10px;min-width:0}
    h1{font-size:29px;line-height:1.45;margin-top:0}h2{font-size:24px;border-top:2px solid #dce5eb;padding-top:28px;margin-top:48px;scroll-margin-top:15px}h3{font-size:19px;margin-top:32px;color:#204e70}
    a{color:#176395;text-underline-offset:3px}p,li{overflow-wrap:anywhere}li{margin:7px 0}strong{color:#173f57}.table-wrap{overflow-x:auto;margin:20px 0}table{border-collapse:collapse;width:100%;font-size:14px;line-height:1.65}th,td{padding:10px 12px;text-align:left;vertical-align:top;border:1px solid #d3dde4;overflow-wrap:anywhere}th{background:#e9f0f5}tr:nth-child(even) td{background:#f8fafb}
    pre{padding:18px;background:#152a38;color:#e7f0f7;border-radius:6px;overflow:auto;font-size:13px;line-height:1.65}code{font-family:"DejaVu Sans Mono",monospace;font-size:.88em}p code,td code,li code{background:#eef2f5;padding:2px 4px;border-radius:3px;overflow-wrap:anywhere}pre code{font-size:inherit}figure{margin:25px 0}img{width:100%;height:auto;display:block}figcaption{text-align:center;color:#526773;font-size:13px;margin-top:10px}button{float:right;padding:6px 12px;cursor:pointer}footer{color:#61717b;font-size:13px;margin-top:45px}
    @media(max-width:1050px){.layout{display:block}nav{position:static;margin-bottom:20px}main{padding:24px 18px}}
    @media print{@page{size:A4;margin:15mm}body{background:white;font-size:10pt}header,nav{display:none}.layout{display:block;margin:0;padding:0}main{padding:0}h1{font-size:22pt}h2{font-size:17pt;break-after:avoid}h3{font-size:13pt;break-after:avoid}table{font-size:8pt}.table-wrap{overflow:visible}pre{white-space:pre-wrap;background:#f0f3f5;color:black;font-size:8pt}figure{break-inside:avoid}img{max-height:235mm;object-fit:contain}a{color:inherit}th{background:#eef2f5}footer{font-size:8pt}}
    '''
    output='<!doctype html><html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>'+html.escape(title)+'</title><style>'+style+'</style></head><body>'
    output+='<header><button onclick="window.print()">인쇄 / PDF 저장</button>2026-11-02 · RUBI Navigation 분석 보고서 · <a href="REPORT.md">Markdown 원본</a> · <a href="PRESENTATION.html">발표 요약</a></header><div class="layout">'+nav+'<main>'+''.join(blocks)
    output+='<footer>보존된 실험 자료와 코드 스냅샷으로 작성. 그림은 이 HTML에 내장되어 있습니다. evidence 링크는 함께 제공된 폴더가 필요합니다.</footer></main></div></body></html>'
    path.with_suffix('.html').write_text(output)


if __name__=='__main__':
    for name in ['REPORT.md','PRESENTATION.md']:
        render(ROOT/name)
