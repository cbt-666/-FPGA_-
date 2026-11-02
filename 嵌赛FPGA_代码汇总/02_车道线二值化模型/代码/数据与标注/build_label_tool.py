#!/usr/bin/env python3
"""生成自包含的浏览器标注工具（单 HTML 文件）

把标注集图片以 base64 嵌入，双击即可用，不需要装任何东西、不需要服务器。
输出到桌面，方便直接用。

操作：
    左线：A = 虚线    S = 实线
    右线：K = 虚线    L = 实线
    两张都判完自动跳到下一张；Z 撤销；进度存 localStorage
    标完点「导出 CSV」
"""
import base64
import json
import os
import sys

# 用法: build_label_tool.py [标注集目录] [输出html]
SRC = sys.argv[1] if len(sys.argv) > 1 else r'D:\ADAS_Datasets\06_车道线_lane\_label_set'
OUT = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\cbt\Desktop\嵌赛FPGA\车道标注工具.html'
TITLE = sys.argv[3] if len(sys.argv) > 3 else '车道虚实标注'
MAXW = 1000


def main():
    import cv2
    import numpy as np

    meta = json.load(open(os.path.join(SRC, 'manifest.json'), encoding='utf-8'))
    items = []
    for m in meta:
        p = os.path.join(SRC, m['img'])
        im = cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)
        if im is None:
            continue
        h, w = im.shape[:2]
        if w > MAXW:
            im = cv2.resize(im, (MAXW, int(h * MAXW / w)))
        ok, buf = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 78])
        if not ok:
            continue
        items.append({'n': m['img'], 's': m['src'],
                      'd': base64.b64encode(buf.tobytes()).decode()})
    print('嵌入 %d 张' % len(items))

    html = """<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>__TITLE__</title>
<style>
*{box-sizing:border-box}
body{margin:0;font-family:"Microsoft YaHei",system-ui,sans-serif;background:#12151a;color:#e6e9ee}
#bar{position:sticky;top:0;background:#1b1f27;padding:10px 16px;display:flex;
 align-items:center;gap:16px;border-bottom:1px solid #2c323d;z-index:9;flex-wrap:wrap}
#bar b{color:#4ea1ff}
.pill{background:#232935;padding:4px 12px;border-radius:20px;font-size:14px}
#wrap{display:flex;justify-content:center;padding:14px}
img{max-width:100%;max-height:62vh;border-radius:8px;border:1px solid #2c323d}
#ctrl{display:flex;justify-content:center;gap:40px;padding:6px 20px 20px;flex-wrap:wrap}
.grp{background:#1b1f27;padding:12px 18px;border-radius:12px;border:1px solid #2c323d}
.grp h3{margin:0 0 10px;font-size:15px;text-align:center}
.grp.L h3{color:#ff5b5b}.grp.R h3{color:#4ade80}
button{font-family:inherit;font-size:16px;padding:10px 22px;margin:0 4px;border-radius:8px;
 border:1px solid #39404d;background:#232935;color:#e6e9ee;cursor:pointer}
button:hover{background:#2c3543}
button.on{background:#2f6fd0;border-color:#4ea1ff}
button.on.dash{background:#c0392b;border-color:#ff6b5b}
button.on.solid{background:#1f8a4c;border-color:#4ade80}
button.on.none{background:#4a5262;border-color:#7c8798}
kbd{background:#0d1015;border:1px solid #39404d;border-radius:4px;padding:0 5px;font-size:12px;color:#9aa4b2}
#hint{text-align:center;color:#8b95a5;font-size:13px;padding-bottom:8px;line-height:1.9}
#exp{background:#2f6fd0;border-color:#4ea1ff;font-weight:bold}
</style></head><body>
<div id="bar">
  <b>__TITLE__</b>
  <span class="pill" id="prog">0 / 0</span>
  <span class="pill" id="cur">—</span>
  <span style="flex:1"></span>
  <button onclick="undo()">撤销 <kbd>Z</kbd></button>
  <button onclick="prev()">上一张 <kbd>&larr;</kbd></button>
  <button onclick="skip()">跳过</button>
  <button id="exp" onclick="exp()">导出 CSV</button>
</div>
<div id="wrap"><img id="im" src=""></div>
<div id="ctrl">
  <div class="grp L"><h3>左边界线</h3>
    <button id="Ld" onclick="set('L','dash')">虚线 <kbd>A</kbd></button>
    <button id="Ls" onclick="set('L','solid')">实线 <kbd>S</kbd></button>
    <button id="Ln" onclick="set('L','none')">无线/看不清 <kbd>D</kbd></button>
  </div>
  <div class="grp R"><h3>右边界线</h3>
    <button id="Rd" onclick="set('R','dash')">虚线 <kbd>K</kbd></button>
    <button id="Rs" onclick="set('R','solid')">实线 <kbd>L</kbd></button>
    <button id="Rn" onclick="set('R','none')">无线/看不清 <kbd>J</kbd></button>
  </div>
</div>
<div id="hint">
  看图上 <span style="color:#ff5b5b">红刻度 L</span> 和 <span style="color:#4ade80">绿刻度 R</span> 标记的车道线 ——
  是<b>断续的虚线</b>还是<b>连续的实线</b>？没有可判的线就选「无线/看不清」。<br>
  两边都判完自动进入下一张 · 进度自动保存 · 随时可导出
</div>
<script>
const ITEMS = __DATA__;
let i = 0, res = {}, advToken = 0;
try { const s = localStorage.getItem('lane_lbl'); if (s) res = JSON.parse(s); } catch(e){}
function save(){ try{ localStorage.setItem('lane_lbl', JSON.stringify(res)); }catch(e){} }
function nextUnlabeled(from){ for(let k=from;k<ITEMS.length;k++){ const r=res[ITEMS[k].n];
  if(!r||!r.L||!r.R) return k; } return -1; }
function show(){
  if(i<0||i>=ITEMS.length){ document.getElementById('im').src='';
    document.getElementById('cur').textContent='全部完成'; return; }
  const it=ITEMS[i];
  document.getElementById('im').src='data:image/jpeg;base64,'+it.d;
  document.getElementById('cur').textContent=it.n+'   ('+(i+1)+'/'+ITEMS.length+')';
  const r=res[it.n]||{};
  for(const k of ['Ld','Ls','Ln','Rd','Rs','Rn']){ const b=document.getElementById(k); b.className=''; }
  if(r.L==='dash') document.getElementById('Ld').className='on dash';
  if(r.L==='solid') document.getElementById('Ls').className='on solid';
  if(r.L==='none') document.getElementById('Ln').className='on none';
  if(r.R==='dash') document.getElementById('Rd').className='on dash';
  if(r.R==='solid') document.getElementById('Rs').className='on solid';
  if(r.R==='none') document.getElementById('Rn').className='on none';
  let done=0; for(const k in res){ if(res[k].L&&res[k].R) done++; }
  document.getElementById('prog').textContent=done+' / '+ITEMS.length;
}
function set(side,val){
  if(i<0||i>=ITEMS.length) return;
  const n=ITEMS[i].n; res[n]=res[n]||{};
  res[n][side]=val; save(); show();
  if(res[n].L&&res[n].R){
    const tok = ++advToken, from = i;
    setTimeout(()=>{
      if(tok !== advToken) return;          // 已被后续操作取代，丢弃
      const k=nextUnlabeled(from+1);
      if(k>=0){ i=k; show(); }
    }, 170);
  } else {
    advToken++;                              // 取消挂起的跳转
  }
}
function skip(){ if(i<ITEMS.length-1){ i++; show(); } }
function prev(){ if(i>0){ i--; show(); } }
function undo(){ if(i>0){ i--; show(); } }
function exp(){
  let out='img,src,L,R\\n';
  for(const it of ITEMS){ const r=res[it.n]; if(!r||!r.L||!r.R) continue;
    out+=it.n+','+it.s+','+r.L+','+r.R+'\\n'; }
  const b=new Blob([out],{type:'text/csv;charset=utf-8'});
  const a=document.createElement('a');
  a.href=URL.createObjectURL(b); a.download='lane_labels.csv'; a.click();
}
document.addEventListener('keydown',e=>{
  const k=e.key.toLowerCase();
  if(k==='a') set('L','dash'); else if(k==='s') set('L','solid');
  else if(k==='d') set('L','none');
  else if(k==='k') set('R','dash'); else if(k==='l') set('R','solid');
  else if(k==='j') set('R','none');
  else if(k==='z') undo(); else if(k==='arrowleft') prev();
});
const k0=nextUnlabeled(0); i = k0>=0?k0:0; show();
</script></body></html>"""

    html = html.replace('__DATA__', json.dumps(items, ensure_ascii=False)).replace('__TITLE__', TITLE)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as fh:
        fh.write(html)
    sz = os.path.getsize(OUT) / 1048576
    print('已生成: %s  (%.1f MB)' % (OUT, sz))


if __name__ == '__main__':
    main()
