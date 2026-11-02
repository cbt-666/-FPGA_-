#!/usr/bin/env python3
"""车道虚实标注工具 v2 —— 带「路口/不用判」键 + 模型预填

相比 v1 的两处改动（都是用户提出的）：
  1. 新增 <kbd>X</kbd> = 路口/不用判变道。用户指出的关键点：
     **只有正常行驶时才存在变道问题**，路口帧混进训练/评估会误判准确率。
  2. **模型预填**：每条边界预先选中模型预测值（按钮带虚线框，表示未确认）。
     用户只需在错的地方改 —— 比从零标快得多。

用法:
  build_label_tool2.py <标注集目录> <输出html> [标题] [预填json]
"""
import sys
import os
import json
import base64

SRC = sys.argv[1] if len(sys.argv) > 1 else r'D:\ADAS_Datasets\06_车道线_lane\_label_set3'
OUT = sys.argv[2] if len(sys.argv) > 2 else r'C:\Users\cbt\Desktop\嵌赛FPGA\车道标注工具2.html'
TITLE = sys.argv[3] if len(sys.argv) > 3 else '车道变道判断标注'
PREFILL = sys.argv[4] if len(sys.argv) > 4 else ''
MAXW = 1100


def main():
    import cv2
    import numpy as np

    meta = json.load(open(os.path.join(SRC, 'manifest.json'), encoding='utf-8'))
    pre = {}
    if PREFILL and os.path.exists(PREFILL):
        pre = json.load(open(PREFILL, encoding='utf-8'))
        print('预填 %d 条' % len(pre))

    items = []
    for m in meta:
        p = os.path.join(SRC, m['img'])
        im = cv2.imdecode(np.fromfile(p, np.uint8), cv2.IMREAD_COLOR)
        if im is None:
            continue
        h, w = im.shape[:2]
        if w > MAXW:
            im = cv2.resize(im, (MAXW, int(h * MAXW / w)))
        ok, buf = cv2.imencode('.jpg', im, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if not ok:
            continue
        items.append({'n': m['img'], 's': m['src'],
                      'd': base64.b64encode(buf.tobytes()).decode(),
                      'p': pre.get(m['img'], {})})
    print('嵌入 %d 张' % len(items))

    html = r"""<!DOCTYPE html>
<html lang="zh"><head><meta charset="utf-8">
<title>__TITLE__</title>
<style>
*{box-sizing:border-box}
body{margin:0;font-family:"Microsoft YaHei",system-ui,sans-serif;background:#12151a;color:#e6e9ee}
#bar{position:sticky;top:0;background:#1b1f27;padding:10px 16px;display:flex;
 align-items:center;gap:14px;border-bottom:1px solid #2c323d;z-index:9;flex-wrap:wrap}
#bar b{color:#4ea1ff}
.pill{background:#232935;padding:4px 12px;border-radius:20px;font-size:14px}
#wrap{display:flex;justify-content:center;padding:10px}
img{max-width:100%;max-height:56vh;border-radius:8px;border:1px solid #2c323d}
#ctrl{display:flex;justify-content:center;gap:16px;padding:6px 16px 14px;flex-wrap:wrap}
.grp{background:#1b1f27;padding:10px 16px;border-radius:12px;border:1px solid #2c323d}
.grp h3{margin:0 0 8px;font-size:14px;text-align:center;color:#9aa6b8}
button{background:#232935;color:#dfe5ee;border:1px solid #333c4a;border-radius:8px;
 padding:8px 14px;font-size:14px;cursor:pointer;margin:0 3px;font-family:inherit}
button:hover{background:#2c3444}
button.on.dash{background:#14532d;border-color:#4ade80;color:#dcfce7}
button.on.solid{background:#7f1d1d;border-color:#f87171;color:#fee2e2}
button.on.none{background:#3f3f46;border-color:#a1a1aa;color:#f4f4f5}
/* 预填 = 虚线框，提示"还没确认" */
button.pre.dash{border:2px dashed #4ade80}
button.pre.solid{border:2px dashed #f87171}
button.pre.none{border:2px dashed #a1a1aa}
kbd{background:#0d1015;border:1px solid #374151;border-radius:4px;padding:1px 5px;
 font-size:12px;color:#9aa6b8}
#hint{padding:8px 20px 24px;font-size:13px;color:#93a0b4;line-height:1.7;text-align:center}
#big{font-size:20px;font-weight:600;padding:6px 16px;border-radius:8px;background:#232935}
.xmark{background:#78350f !important;border-color:#fbbf24 !important;color:#fef3c7 !important}
</style></head><body>
<div id="bar">
  <b>__TITLE__</b>
  <span class="pill" id="cur"></span>
  <span class="pill" id="prog"></span>
  <button onclick="prev()">← 上一张</button>
  <button onclick="skip()">跳过</button>
  <button id="xbtn" onclick="markX()">路口/不用判 <kbd>X</kbd></button>
  <button id="exp" onclick="exp()">导出 CSV</button>
</div>
<div id="wrap"><img id="im" src=""></div>
<div id="ctrl">
  <div class="grp L"><h3>自车车道 · 左边界</h3>
    <button id="Ld" onclick="set('L','dash')">虚线 <kbd>A</kbd></button>
    <button id="Ls" onclick="set('L','solid')">实线 <kbd>S</kbd></button>
    <button id="Ln" onclick="set('L','none')">无线/看不清 <kbd>D</kbd></button>
  </div>
  <div class="grp R"><h3>自车车道 · 右边界</h3>
    <button id="Rd" onclick="set('R','dash')">虚线 <kbd>K</kbd></button>
    <button id="Rs" onclick="set('R','solid')">实线 <kbd>L</kbd></button>
    <button id="Rn" onclick="set('R','none')">无线/看不清 <kbd>J</kbd></button>
  </div>
</div>
<div id="hint">
  看<b>自车所在车道</b>的左右两条边界线，各自是<b>虚线</b>还是<b>实线</b>（近处/远处都看，以能看清的为准）。<br>
  <b style="color:#fbbf24">如果这帧是路口/红绿灯/斑马线，或者当前根本不需要变道判断 —— 直接按 <kbd>X</kbd> 跳过</b>，不要标。<br>
  <span style="color:#4ea1ff">虚线框 = 模型预填值，仅供参考，需你确认或改</span> · 两边判完自动进入下一张 · 进度自动保存
</div>
<script>
const ITEMS = __DATA__;
let i = 0, res = {}, advToken = 0;
try { const s = localStorage.getItem('lane_lbl2'); if (s) res = JSON.parse(s); } catch(e){}
function save(){ try{ localStorage.setItem('lane_lbl2', JSON.stringify(res)); }catch(e){} }
function done(r){ return r && (r.X || (r.L && r.R)); }
function nextUnlabeled(from){ for(let k=from;k<ITEMS.length;k++){ if(!done(res[ITEMS[k].n])) return k; } return -1; }
function show(){
  if(i<0||i>=ITEMS.length){ document.getElementById('im').src='';
    document.getElementById('cur').textContent='全部完成'; return; }
  const it=ITEMS[i];
  document.getElementById('im').src='data:image/jpeg;base64,'+it.d;
  document.getElementById('cur').textContent=it.n+'   ('+(i+1)+'/'+ITEMS.length+')';
  const r=res[it.n]||{};
  const p=it.p||{};
  document.getElementById('xbtn').className = r.X ? 'xmark' : '';
  for(const k of ['Ld','Ls','Ln','Rd','Rs','Rn']){ const b=document.getElementById(k);
    b.className=''; b.textContent = b.textContent.replace(' (预)',''); }
  const map={Ld:['L','dash'],Ls:['L','solid'],Ln:['L','none'],
             Rd:['R','dash'],Rs:['R','solid'],Rn:['R','none']};
  for(const id in map){
    const [side,val]=map[id];
    if(r[side]===val) document.getElementById(id).className='on '+val;
    else if(!r[side] && p[side]===val){
      document.getElementById(id).className='pre '+val;
      document.getElementById(id).textContent += ' (预)';
    }
  }
  let n=0; for(const k in res){ if(done(res[k])) n++; }
  document.getElementById('prog').textContent=n+' / '+ITEMS.length;
}
function advance(from){ const k=nextUnlabeled(from+1); if(k>=0){ i=k; show(); } }
function set(side,val){
  if(i<0||i>=ITEMS.length) return;
  const n=ITEMS[i].n; res[n]=res[n]||{};
  res[n][side]=val; delete res[n].X; save(); show();
  if(res[n].L&&res[n].R){
    const tok = ++advToken, from = i;
    setTimeout(()=>{ if(tok!==advToken) return; advance(from); }, 170);
  } else { advToken++; }
}
function markX(){
  if(i<0||i>=ITEMS.length) return;
  const n=ITEMS[i].n; res[n]=res[n]||{};
  res[n].X=true; delete res[n].L; delete res[n].R; save(); show();
  const tok = ++advToken, from = i;
  setTimeout(()=>{ if(tok!==advToken) return; advance(from); }, 140);
}
function skip(){ if(i<ITEMS.length-1){ i++; show(); } }
function prev(){ if(i>0){ i--; show(); } }
function exp(){
  let out='img,src,L,R,X\n';
  for(const it of ITEMS){ const r=res[it.n]; if(!done(r)) continue;
    out+=it.n+','+it.s+','+(r.L||'')+','+(r.R||'')+','+(r.X?1:0)+'\n'; }
  const b=new Blob([out],{type:'text/csv;charset=utf-8'});
  const a=document.createElement('a');
  a.href=URL.createObjectURL(b); a.download='lane_labels2.csv'; a.click();
}
document.addEventListener('keydown',e=>{
  const k=e.key.toLowerCase();
  if(k==='a') set('L','dash'); else if(k==='s') set('L','solid');
  else if(k==='d') set('L','none');
  else if(k==='k') set('R','dash'); else if(k==='l') set('R','solid');
  else if(k==='j') set('R','none');
  else if(k==='x') markX();
  else if(k==='arrowleft') prev();
});
const k0=nextUnlabeled(0); i = k0>=0?k0:0; show();
</script></body></html>"""

    html = html.replace('__DATA__', json.dumps(items, ensure_ascii=False)).replace('__TITLE__', TITLE)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    with open(OUT, 'w', encoding='utf-8') as fh:
        fh.write(html)
    print('已生成: %s  (%.1f MB)' % (OUT, os.path.getsize(OUT) / 1048576))


if __name__ == '__main__':
    main()
