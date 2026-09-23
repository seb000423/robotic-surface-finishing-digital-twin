# -*- coding: utf-8 -*-
"""Pretendard 서브셋 재생성 — 사이트에 실제로 뜨는 글자 기준.

    python scripts/font-subset.py

카피를 고쳤으면 다시 돌린다. 서브셋에 없는 글자가 하나라도 화면에 나오면
--f-sans 폴백으로 Pretendard Full(580~613 KB)이 통째로 내려온다.
글자 수집 범위: index · sub · monitor · 번들 3장의 template.html(주석 제외)
+ 기존 서브셋에 있던 글자(안전망) + ASCII 전체.
"""
import os, re, sys, html
from fontTools.ttLib import TTFont
from fontTools import subset

sys.stdout.reconfigure(encoding='utf-8')
# 정적 파일은 frontend/ 아래로 내려갔다.
# 저장소 루트가 아니라 프론트 루트를 기준으로 잡는다.
ROOT = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "frontend")
FONTS = os.path.join(ROOT, 'assets', 'fonts')

SRC = ['index.html', 'sub.html', 'monitor.html', 'admin.html',
       os.path.join('console-src', 'template.html'),
       os.path.join('library-src', 'template.html'),
       os.path.join('save-src', 'template.html')]


def visible(t):
    t = re.sub(r'<!--.*?-->', ' ', t, flags=re.S)
    t = re.sub(r'/\*.*?\*/', ' ', t, flags=re.S)
    t = re.sub(r'(?m)^\s*//.*$', ' ', t)
    t = re.sub(r'<style\b.*?</style>', ' ', t, flags=re.S | re.I)
    t = re.sub(r'<[^>]+>', ' ', t)
    return html.unescape(t)


chars = set(chr(c) for c in range(0x20, 0x7F))
for rel in SRC:
    p = os.path.join(ROOT, rel)
    if os.path.exists(p):
        chars |= set(visible(open(p, encoding='utf-8').read()))
chars = {c for c in chars if ord(c) >= 0x20 and c not in '  '}

def cut(src, cps):
    opt = subset.Options()
    opt.layout_features = ['*']
    opt.name_IDs = ['*']
    opt.notdef_outline = True
    font = TTFont(src)
    have = set(font.getBestCmap().keys())
    sb = subset.Subsetter(opt)
    sb.populate(unicodes=sorted(cps & have))
    sb.subset(font)
    font.flavor = None
    return font, cps - have


for w in (300, 500, 700):
    src = os.path.join(FONTS, f'Pretendard-{w}.full-ko.woff2')
    dst = os.path.join(FONTS, f'Pretendard-{w}.subset.woff2')
    want = {ord(c) for c in chars}
    # full-ko 는 한글 전용이라 ·–—…°µ×℃ 같은 기호 글리프가 없다.
    # 그 글리프는 기존 서브셋(원본 Pretendard 에서 뽑은 것)이 유일한 공급원이다 —
    # 기존 서브셋을 통째로 살리고(안전망 겸), 새로 필요한 한글만 full-ko 에서 잘라 합친다.
    donor = TTFont(dst)
    have_old = set(donor.getBestCmap().keys())
    new_font, missing = cut(src, want - have_old)
    still = missing - have_old
    tmp_new = os.path.join(FONTS, f'_tmp_new_{w}.ttf')
    tmp_old = os.path.join(FONTS, f'_tmp_old_{w}.ttf')
    new_font.save(tmp_new)
    donor.flavor = None; donor.save(tmp_old)
    from fontTools.merge import Merger
    merged = Merger().merge([tmp_old, tmp_new])
    merged.flavor = 'woff2'
    merged.save(dst)
    os.remove(tmp_new); os.remove(tmp_old)
    n = len(TTFont(dst).getBestCmap())
    print(f'{os.path.basename(dst)}  {n}자  {os.path.getsize(dst)/1024:.1f} KB'
          + (f'  ※ 어디에도 없는 글자 {len(still)}: ' + ''.join(chr(c) for c in sorted(still)) if still else ''))
