"""api_node 가 서빙하는 운용 콘솔 (Robot Admin 형식).

AutoXing 의 rb-admin 과 같은 구성 — 좌측 사이드바 + 섹션별 패널.
구현된 섹션은 실제 데이터를 보여주고, 아직 없는 섹션은
'무엇이 있어야 되는지'를 명시한 자리표시자로 남긴다.
"""

PAGE = r"""<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>UND_AMR Robot Admin</title>
<style>
:root{--bg:#0f1116;--side:#141821;--panel:#181b23;--panel2:#212530;--line:#2a3140;
  --fg:#e8eaf0;--dim:#8f97a8;--accent:#4da3ff;--ok:#3ddc97;--warn:#ffb454;--bad:#ff5c5c;
  font-family:system-ui,-apple-system,"Segoe UI",Roboto,"Noto Sans KR",sans-serif}
*{box-sizing:border-box}
body{margin:0;background:var(--bg);color:var(--fg);font-size:14.5px;
  display:grid;grid-template-columns:212px 1fr;min-height:100vh}

aside{background:var(--side);border-right:1px solid var(--line);
  display:flex;flex-direction:column;position:sticky;top:0;height:100vh;overflow-y:auto}
.brand{padding:16px 16px 12px;border-bottom:1px solid var(--line)}
.brand b{font-size:16px;letter-spacing:-.01em}
.brand div{font-size:11px;color:var(--dim);margin-top:3px}
nav{padding:8px 0;flex:1}
.navgrp{font-size:10px;color:#5c6577;text-transform:uppercase;letter-spacing:.08em;
  padding:12px 16px 5px;font-weight:700}
.nav{display:flex;align-items:center;gap:9px;padding:8px 16px;cursor:pointer;
  color:var(--dim);font-size:13.5px;border-left:2px solid transparent}
.nav:hover{color:var(--fg);background:#1a1f2a}
.nav.sel{color:var(--fg);background:#1c2230;border-left-color:var(--accent)}
.nav .dot{width:6px;height:6px;border-radius:50%;background:#3a4152;flex:none}
.nav .dot.on{background:var(--ok)} .nav .dot.off{background:#3a4152}
.nav .dot.bad{background:var(--bad)}

main{display:flex;flex-direction:column;min-width:0}
header{position:sticky;top:0;z-index:20;background:var(--panel);
  border-bottom:1px solid var(--line);padding:9px 18px;display:flex;
  gap:9px;align-items:center;flex-wrap:wrap}
h1{font-size:15px;margin:0;font-weight:650}
.sp{flex:1}
.chip{font-size:11.5px;padding:4px 10px;border-radius:999px;background:var(--panel2);
  color:var(--dim);border:1px solid var(--line);white-space:nowrap}
.chip.ok{color:var(--ok);border-color:#2b6b52}
.chip.bad{color:var(--bad);border-color:#6b2b2b}
.chip.warn{color:var(--warn);border-color:#6b5426}
.estop{background:var(--bad);color:#fff;border:0;border-radius:9px;
  padding:9px 18px;font-size:14px;font-weight:800;cursor:pointer}
.estop.armed{background:#343a47;color:var(--bad);outline:2px solid var(--bad)}

.wrap{padding:16px 18px;display:none;gap:14px;grid-template-columns:repeat(2,1fr)}
#pane-dash.sel{grid-template-columns:minmax(0,1.9fr) minmax(280px,1fr)}
.wrap.sel{display:grid}
section{background:var(--panel);border:1px solid var(--line);border-radius:11px;padding:14px}
section.full{grid-column:1/-1}
h2{font-size:11.5px;margin:0 0 11px;color:var(--dim);font-weight:650;
  text-transform:uppercase;letter-spacing:.06em}
table{width:100%;border-collapse:collapse;font-variant-numeric:tabular-nums}
td,th{padding:6px 5px;border-bottom:1px solid var(--line);font-size:13px;text-align:right}
td:first-child,th:first-child{text-align:left;color:var(--dim)}
th{font-size:10.5px;color:#6f7788;font-weight:650;text-transform:uppercase}
tbody td:first-child{color:var(--dim)}
canvas,img.cam{width:100%;height:auto;display:block;border-radius:8px;background:#080a0e}
/* 라이다·카메라 패널은 양쪽 열을 다 쓰는 .full 안에 있어, 화면이 넓으면
   폭을 따라 높이까지 같이 커진다(1600px 폭이면 높이 1100px 넘음).
   지도처럼 넓게 볼 필요가 없는 화면이므로 상한을 둔다. */
#cvLidar{max-width:760px;margin:0 auto}
/* 카메라 영상 위에 인식 결과를 겹쳐 그린다 */
.camwrap{position:relative;max-width:840px;margin:0 auto}
/* 카메라 두 대를 한 줄에 반반 (2026-11-13) */
.camrow{display:grid;grid-template-columns:1fr 1fr;gap:10px;margin-top:12px}
.camrow .camcell{min-width:0}
.camrow .camlbl{font-size:12.5px;color:var(--dim);margin:0 0 5px}
.camrow .camwrap{max-width:none;margin:0}
.camrow img.cam{aspect-ratio:4/3;object-fit:contain;width:100%}
@media (max-width:640px){ .camrow{grid-template-columns:1fr} }
.camwrap .cam{max-width:100%}
.camov{position:absolute;left:0;top:0;width:100%;height:100%;pointer-events:none;
  background:transparent}
img.cam{max-width:840px;margin:0 auto}
/* 지도 미리보기는 원본이 수십 px 라 폭 100% 로 늘리면 수 배 확대되어 뭉개진다.
   고정 높이 상자 안에 비율을 유지해 넣고, 확대는 픽셀 그대로 보이게 한다. */
.prevbox{background:#080a0e;border:1px solid var(--line);border-radius:8px;
  height:300px;display:flex;align-items:center;justify-content:center;
  overflow:hidden;padding:8px}
.prevbox img{max-width:100%;max-height:100%;width:auto;height:auto;
  image-rendering:pixelated;display:block}
.prevbox .empty{color:var(--dim);font-size:12.5px}
.pill{display:inline-block;padding:1px 7px;border-radius:9px;font-size:11px;
 border:1px solid var(--line);color:var(--dim);white-space:nowrap}
.pill.ok{border-color:var(--ok);color:var(--ok);background:rgba(64,200,120,.10)}
.maprow{cursor:pointer}
.maprow:hover td{background:#1a1f2a}
.maprow.sel td{background:#1c2230}
.maprow.sel td:first-child{color:var(--accent)}
button{background:var(--panel2);color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:8px 13px;font-size:13px;cursor:pointer}
button:hover{border-color:var(--accent)}
.row{display:flex;gap:7px;flex-wrap:wrap;align-items:center}
.pad{display:grid;grid-template-columns:repeat(3,1fr);gap:7px;margin-top:9px}
.pad button{padding:15px 0;font-size:18px;user-select:none;touch-action:none}
.pad button.on,.pad button:active{background:var(--accent);color:#06101c;border-color:var(--accent)}
.pad .blank{visibility:hidden}
.slider{display:grid;grid-template-columns:52px 1fr 58px;gap:8px;align-items:center;margin:7px 0}
.slider label{color:var(--dim);font-size:12px}
input[type=range]{width:100%;accent-color:var(--accent)}
.val{text-align:right;font-weight:650;font-variant-numeric:tabular-nums;font-size:13px}
.seg{display:flex;gap:6px}
.seg button{flex:1;color:var(--dim);font-weight:650}
.seg button.sel{background:var(--accent);color:#06101c;border-color:var(--accent)}
.note{color:var(--dim);font-size:12px;line-height:1.65;margin-top:9px}
pre{background:#0a0d12;border:1px solid var(--line);border-radius:8px;padding:10px;
  margin:0;max-height:420px;overflow:auto;font-family:ui-monospace,Menlo,monospace;
  font-size:12px;line-height:1.55;white-space:pre-wrap;word-break:break-all}
.todo{border:1px dashed #3a4356;border-radius:10px;padding:20px;text-align:center;
  color:var(--dim);line-height:1.8}
.todo b{color:var(--warn);display:block;margin-bottom:6px;font-size:14px}
.modeseg{display:flex;gap:0;margin-bottom:12px;border-radius:9px;overflow:hidden;
  border:1px solid var(--line)}
.modeseg button{flex:1;border:0;border-radius:0;background:var(--panel2);
  color:var(--dim);font-weight:700;font-size:12.5px;letter-spacing:.04em;padding:10px 0}
.modeseg button+button{border-left:1px solid var(--line)}
.modeseg button.sel{background:var(--accent);color:#06101c}
.act{width:100%;padding:12px 0;font-size:13.5px;font-weight:700;letter-spacing:.02em;
  border-radius:9px;margin-bottom:9px}
.act.primary{background:var(--ok);color:#06231a;border-color:var(--ok)}
.act.primary:hover{filter:brightness(1.08)}
.act.danger{background:var(--bad);color:#fff;border-color:var(--bad)}
.act.danger:hover{filter:brightness(1.08)}
.act:disabled{opacity:.38;cursor:not-allowed}
.act:disabled:hover{border-color:var(--line)}
.actrow{display:grid;grid-template-columns:repeat(3,1fr);gap:8px}
.actrow.g2{grid-template-columns:repeat(2,1fr)}
.actrow .act{margin-bottom:9px}
.actmsg{min-height:17px;font-size:12px;color:var(--dim);margin:2px 0 4px}
.joybox{display:none;border-top:1px solid var(--line);margin-top:6px;padding-top:12px}
.joybox.on{display:block}
#joy{width:100%;max-width:260px;margin:0 auto 6px;display:block;
  background:transparent;touch-action:none;cursor:grab}
#joy.drag{cursor:grabbing}
.joyhint{text-align:center;color:var(--dim);font-size:12px;margin-bottom:8px;
  font-variant-numeric:tabular-nums}
.mapcard{grid-column:1/2}
.maphead{display:flex;align-items:center;gap:9px;margin-bottom:10px;flex-wrap:wrap}
.maphead button{padding:5px 11px;font-size:13px}
.tg{display:flex;align-items:center;gap:5px;font-size:12.5px;color:var(--dim);
  cursor:pointer;user-select:none}
.tg input{accent-color:var(--accent);cursor:pointer}
#cvMap{cursor:grab} #cvMap.drag{cursor:grabbing}
.maphint{color:var(--dim);font-size:12px;margin-top:8px;line-height:1.6}
.lv-WARN{color:var(--warn)} .lv-ERROR,.lv-FATAL{color:var(--bad)} .lv-INFO{color:var(--dim)}
.bad-txt{color:var(--bad)} .ok-txt{color:var(--ok)} .warn-txt{color:var(--warn)}
.dim-txt{color:var(--dim)}
@media(max-width:1000px){body{grid-template-columns:1fr}aside{position:static;height:auto}
  .wrap.sel{grid-template-columns:1fr}}
.relocbar{display:none;gap:8px;align-items:center;padding:8px 10px;
  border-bottom:1px solid var(--line);background:#141821;flex-wrap:wrap}
.relocbar.on{display:flex}
.rb{background:#1b2130;color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:7px 14px;font-size:13px;cursor:pointer}
.relocbar .rb{background:#1b2130;color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:7px 14px;font-size:13px;cursor:pointer}
.relocbar .rb.sel{background:#2a63a8;border-color:#3c7fd0}
.relocbar .rb.go{background:#1f6f4a;border-color:#2c9160}
.relocbar .rbmsg{color:var(--dim);font-size:12px;margin-left:4px}
.relocbar .nud{display:inline-flex;gap:3px;flex-wrap:wrap}
.relocbar .nud .rb{padding:4px 7px;min-width:28px}
.mdlbg{position:fixed;inset:0;background:rgba(0,0,0,.6);display:none;
  align-items:center;justify-content:center;z-index:50}
.mdlbg.on{display:flex}
.mdl{background:var(--side);border:1px solid var(--line);border-radius:14px;
  padding:22px 24px;min-width:380px;max-width:92vw;box-shadow:0 18px 50px rgba(0,0,0,.5)}
.mdl h3{margin:0 0 10px;font-size:16px}
.mdl p{margin:0 0 16px;color:var(--dim);font-size:13px;line-height:1.6}
.mdl input{width:100%;background:#0d0f14;color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:10px 12px;font-size:14px;margin-bottom:14px}
.mdlrow{display:flex;gap:8px;flex-wrap:wrap}
.mdlrow .act{margin:0;flex:1;min-width:120px}
/* 지점 이동 (대시보드, 2026-09-29) */
.mvgrid{display:grid;grid-template-columns:repeat(auto-fill,minmax(96px,1fr));gap:7px;margin-top:10px}
.mvgrid button{padding:11px 6px;font-weight:600;overflow:hidden;text-overflow:ellipsis;white-space:nowrap}
.mvgrid button.sel{background:var(--accent);color:#06101c;border-color:var(--accent)}
.mvrow{display:grid;grid-template-columns:44px 1fr;gap:8px;align-items:center;margin-top:9px}
.mvrow label.k{color:var(--dim);font-size:12.5px}
.mvrow select,.mvrow input[type=number]{background:#1b2130;color:var(--fg);border:1px solid var(--line);
  border-radius:8px;padding:7px 9px;font-size:13px;width:100%;min-width:0}
.mvrow .inl{display:flex;gap:9px;align-items:center}
.mvrow .inl input[type=number]{width:96px}
.mvrow input:disabled{opacity:.4}
.mvstate{margin-top:10px;padding:9px 11px;border-radius:9px;background:var(--panel2);
  border:1px solid var(--line);font-size:13px;line-height:1.55;display:none}
.mvstate.on{display:block}
.mvstate b{font-weight:650}
/* 배터리 (타보스 BMS, 2026-09-29) */
.batchip{display:inline-flex;align-items:center;gap:6px}
.bicon{width:20px;height:10px;border:1.5px solid currentColor;border-radius:2px;
  position:relative;display:inline-block;flex:none}
.bicon::after{content:'';position:absolute;right:-4px;top:2px;width:2px;height:3px;
  background:currentColor;border-radius:0 1px 1px 0}
.bicon i{position:absolute;left:1px;top:1px;bottom:1px;background:currentColor;border-radius:1px}
.bathero{display:flex;gap:22px;align-items:center;flex-wrap:wrap}
.bigsoc{font-size:52px;font-weight:650;line-height:1;letter-spacing:-.02em}
.bigsoc small{font-size:22px;color:var(--dim);font-weight:600;margin-left:2px}
.batsub{color:var(--dim);font-size:13px;margin-top:6px}
.meter{height:12px;background:#1d2a3d;border-radius:6px;overflow:hidden}
.meter i{display:block;height:100%;border-radius:6px}
.meter i.ok{background:var(--ok)} .meter i.warn{background:var(--warn)} .meter i.bad{background:var(--bad)}
.tiles{display:grid;grid-template-columns:repeat(auto-fill,minmax(138px,1fr));gap:10px;margin-top:14px}
.tile{background:var(--panel2);border:1px solid var(--line);border-radius:9px;padding:9px 11px}
.tile .k{font-size:11.5px;color:var(--dim)}
.tile .v{font-size:19px;font-weight:600;margin-top:3px}
.tile .v small{font-size:12px;color:var(--dim);font-weight:500;margin-left:2px}
.bflags{display:flex;gap:6px;flex-wrap:wrap;margin-top:10px}
.bcharts{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:14px 18px}
.bchart h3{font-size:12.5px;color:var(--dim);margin:0 0 6px;font-weight:600}
.bwrap{position:relative}
.bwrap canvas{height:170px;background:transparent;border-radius:0;touch-action:pan-y}
.btip{position:absolute;pointer-events:none;background:#0b0e14;border:1px solid var(--line);
  border-radius:7px;padding:6px 9px;font-size:12px;white-space:nowrap;display:none;z-index:5;
  font-variant-numeric:tabular-nums;line-height:1.55}
.btip b{color:var(--fg)} .btip span{color:var(--dim)}
.bempty{position:absolute;inset:0;display:flex;align-items:center;justify-content:center;
  color:var(--dim);font-size:12.5px}
details.btab summary{cursor:pointer;color:var(--dim);font-size:12.5px;margin-top:10px}
details.btab table{margin-top:8px}
@media(max-width:760px){.bcharts{grid-template-columns:1fr}.bigsoc{font-size:44px}}
</style></head><body>

<aside>
  <div class="brand"><b>UND_AMR</b><div id="bSn">Robot Admin</div></div>
  <nav id="nav"></nav>
</aside>

<script>
// ?shot=3000 으로 열면 load 이벤트를 그만큼 늦춘다(헤드리스 캡처용).
(function(){ var m=/[?&]shot=(\d+)/.exec(location.search);
  if(m) document.write('<img src="/ready?ms='+m[1]+'" style="display:none">'); })();
</script>
<main>
  <header>
    <h1 id="hTitle">대시보드</h1>
    <span class="chip" id="cLink">확인 중</span>
    <span class="chip" id="cMode">-</span>
    <span class="chip" id="cMotor">-</span>
    <span class="chip" id="cBatt">-</span>
    <span class="sp"></span>
    <button class="estop" id="btnEstop" onclick="toggleEstop()">■ 비상정지</button>
  </header>
  <div id="panes"></div>
</main>

<!-- 지도 수정 '초기화' 확인 창. 이미 적용해 둔 것까지 지우는 일이라
     되돌릴 수 없다. 실제로 이걸 모르고 눌러 편집을 통째로 날린 적이 있다. -->
<!-- 도킹 창. 등록된 대차 POI 를 늘어놓고 하나를 골라 진입시킨다.
     대차 자리는 POI 가 알고 있으므로, 여기서는 '어느 렉인가' 만 고른다.
     2026-11-12 충전소 POI 도 같이 늘어놓는다 (후진 도킹 전까지는 대기 자리로 이동). -->
<div class="mdlbg" id="dockBg">
 <div class="mdl" style="min-width:300px">
  <h3>도킹</h3>
  <div id="dockList"></div>
  <div id="dockJob" class="actmsg" style="margin:12px 0 0"></div>
  <div class="mdlrow" style="margin-top:14px">
    <button class="act danger" id="dockStop" onclick="poiJobCancel()"
      style="display:none">중단</button>
    <button class="act" onclick="dockClose()">닫기</button>
  </div>
 </div>
</div>

<div class="mdlbg" id="edResetBg">
 <div class="mdl">
  <h3>지도 수정을 전부 지웁니다</h3>
  <p>이미 적용해 둔 <b id="edResetN"></b> 이 함께 지워지고 지도가 원본으로
     돌아갑니다. 지운 자리 목록은 되살릴 수 없습니다.</p>
  <div class="mdlrow">
    <button class="act danger" onclick="editResetGo()">전부 지우기</button>
    <button class="act" onclick="editResetClose()">돌아가기</button>
  </div>
 </div>
</div>

<!-- 매핑 종료 시 저장 여부를 묻는 창. 브라우저 confirm 대신 쓴다.
     버튼 이름만으로 결과가 분명해야 실수로 지도를 날리지 않는다. -->
<div class="mdlbg" id="stopBg">
 <div class="mdl">
  <!-- 이어서 매핑한 경우. 새 이름을 묻는 것이 아니라 원래 지도에
       덮어쓰는 것이 기본이다 - 그 지도를 넓히려고 이어 그린 것이므로. -->
  <div id="stopStep0" style="display:none">
   <h3>매핑을 종료합니다</h3>
   <p><b id="stopBase"></b> 에 이어서 그렸습니다.<br>
      덮어써서 저장하면 늘어난 부분이 그 지도에 합쳐집니다.
      저장하지 않으면 이번에 덧그린 부분만 버리고 원래 지도로 돌아갑니다.<br>
      <span style="color:var(--warn)">덮어쓰면 그 지도의 이전 주행 기록은
      이번 것으로 바뀝니다.</span></p>
   <div class="mdlrow">
     <button class="act primary" onclick="stopSaveBase()">덮어써서 저장하고 종료</button>
     <button class="act" onclick="stopGoName()">다른 이름으로 저장</button>
     <button class="act danger" onclick="stopDiscard()">저장 안 하고 종료</button>
     <button class="act" onclick="stopClose()">돌아가기</button>
   </div>
  </div>
  <div id="stopStep1">
   <h3>매핑을 종료합니다</h3>
   <p>지금까지 작성한 지도를 저장할까요?<br>
      저장하지 않으면 이 매핑 내용은 사라집니다.</p>
   <div class="mdlrow">
     <button class="act primary" onclick="stopGoName()">저장하기</button>
     <button class="act danger" onclick="stopDiscard()">저장 안 하고 종료</button>
     <button class="act" onclick="stopClose()">돌아가기</button>
   </div>
  </div>
  <div id="stopStep2" style="display:none">
   <h3>지도 이름</h3>
   <p>저장할 이름을 넣으세요. 같은 이름이 있으면 덮어씁니다.</p>
   <input id="stopName" placeholder="예: office_1f">
   <div class="mdlrow">
     <button class="act primary" onclick="stopSaveGo()">저장하고 종료</button>
     <button class="act danger" onclick="stopDiscard()">저장 안 하고 종료</button>
     <button class="act" onclick="stopClose()">돌아가기</button>
   </div>
  </div>
  <div id="stopMsg" class="actmsg"></div>
 </div>
</div>


<script>
// ---------------------------------------------------------------- 섹션 정의
const SECTIONS = [
  {id:'dash',   grp:'상태', name:'대시보드',   ok:true},
  {id:'motor',  grp:'상태', name:'모터',       ok:true},
  {id:'battery',grp:'상태', name:'배터리',     ok:true},

  {id:'lidar',  grp:'센서', name:'라이다',     ok:true},
  {id:'camera', grp:'센서', name:'카메라',     ok:true},

  {id:'maps',   grp:'내비게이션', name:'맵',    ok:true},
  {id:'mapping',grp:'내비게이션', name:'매핑',  ok:true},
  {id:'moves',  grp:'내비게이션', name:'이동 작업', ok:false,
   need:'nav2 의 NavigateToPose 액션 연동이 필요합니다. 맵이 있어야 의미가 있어 매핑 다음 단계입니다.'},

  {id:'nodes',  grp:'시스템', name:'노드',     ok:true},
  {id:'topics', grp:'시스템', name:'토픽',     ok:true},
  {id:'log',    grp:'시스템', name:'로그',     ok:true},
  {id:'params', grp:'시스템', name:'파라미터', ok:false,
   need:'ros2 param 서비스 호출을 API 에 노출해야 합니다. 지금은 config/amr_params.yaml 을 직접 편집합니다.'},
  {id:'services',grp:'시스템',name:'서비스',   ok:true},
  {id:'device', grp:'시스템', name:'장비 정보', ok:true},
  {id:'cartcal',grp:'시스템', name:'대차 실측',  ok:true},
  {id:'network',grp:'시스템', name:'네트워크',   ok:true},
];

const PANE = {
dash:`<section class="mapcard">
  <div class="maphead">
    <h2 style="margin:0">맵 / 로봇 위치</h2>
    <span class="sp"></span>
    <label class="tg"><input type="checkbox" id="tgScan" checked> 라이다</label>
    <label class="tg"><input type="checkbox" id="tgFollow" checked> 로봇 추적</label>
    <label class="tg"><input type="checkbox" id="tgGrid" checked> 격자</label>
    <button onclick="zoomBy(1.25)">＋</button>
    <button onclick="zoomBy(0.8)">－</button>
    <button onclick="resetView()">맞춤</button>
    <button onclick="goalOpen()">이동</button>
  </div>
  <!-- 렉 진입이 도는 동안만 뜨는 띠. 도킹 창을 닫아도 어디까지 왔는지
       보이도록 지도 위에 둔다. -->
  <div class="relocbar" id="jobBar">
    <b style="font-size:13px">렉 진입</b>
    <span class="rbmsg" id="jbMsg"></span>
    <span class="sp"></span>
    <button class="rb" onclick="poiJobCancel()" style="color:var(--bad)">중단</button>
  </div>
  <div class="relocbar" id="goalBar">
    <b style="font-size:13px">이동 목표</b>
    <span class="sp"></span>
    <button class="rb" onclick="goalCancel()">닫기</button>
    <span class="rbmsg" id="gbMsg"></span>
  </div>
  <div class="relocbar" id="relocBar">
    <b style="font-size:13px">위치 재조정</b>
    <button class="rb" id="rbMove" onclick="relocMode('move')">위치</button>
    <button class="rb" id="rbRot"  onclick="relocMode('rot')">회전</button>
    <button class="rb" onclick="relocAuto()">자동</button>
    <span class="nud" title="미세 이동 (로봇 기준)">
      <button class="rb" onclick="relocNudge(1,0,0)" title="앞">▲</button>
      <button class="rb" onclick="relocNudge(-1,0,0)" title="뒤">▼</button>
      <button class="rb" onclick="relocNudge(0,1,0)" title="왼쪽">◀</button>
      <button class="rb" onclick="relocNudge(0,-1,0)" title="오른쪽">▶</button>
      <button class="rb" onclick="relocNudge(0,0,1)" title="왼쪽으로 돌기">⟲</button>
      <button class="rb" onclick="relocNudge(0,0,-1)" title="오른쪽으로 돌기">⟳</button>
      <button class="rb" data-big="0" onclick="relocStep(this)" title="한 번에 움직이는 양">1cm·0.5°</button>
    </span>
    <button class="rb" onclick="relocRelease()" title="수동 고정을 풀고 라이다로 다시 맞춥니다">보정 켜기</button>
    <span class="sp"></span>
    <button class="rb go" onclick="relocConfirm()">확인</button>
    <button class="rb" onclick="relocCancel()">취소</button>
    <span class="rbmsg" id="rbMsg"></span>
  </div>
  <canvas id="cvMap" width="960" height="620"></canvas>
  <div class="maphint" id="mapHint"></div>
  <div class="camrow">
    <div class="camcell"><div class="camlbl">정면 카메라 — 렉 인식</div>
      <div class="camwrap"><img id="imgCamD" class="cam" alt="정면 카메라">
        <canvas id="cvCamD" class="camov"></canvas></div></div>
    <div class="camcell"><div class="camlbl">아래 카메라</div>
      <div class="camwrap"><img id="imgCamDn" class="cam" alt="아래 카메라"></div></div>
  </div></section>
 <div style="display:grid;gap:14px;align-content:start">
  <section>
   <div class="modeseg">
     <button id="mAuto" onclick="setMode('auto')">자동</button>
     <button id="mMan"  onclick="setMode('manual')">수동</button>
   </div>

   <button class="act primary" onclick="act('enable')">모터 사용 (WAKE UP)</button>

   <div class="actrow">
     <button class="act" onclick="dockOpen()">도킹</button>
     <button class="act danger" onclick="act('stop')">정지</button>
     <button class="act" onclick="jobExit()" title="도킹한 렉에서 렉 축을 따라 앞으로 곧게 나온다">렉 나오기</button>
   </div>
   <div class="actrow g2">
     <button class="act" disabled title="맵을 만든 뒤 사용할 수 있습니다">맵 변경</button>
     <button class="act" disabled title="rosbag 기록 연동 필요">기록 저장</button>
   </div>
   <div class="actrow g2">
     <button class="act" onclick="act('release')">축 해제</button>
     <button class="act" onclick="act('clear_errors')">알람 클리어</button>
   </div>
   <div class="actrow">
     <button class="act" onclick="act('lift_up')">리프트 올림</button>
     <button class="act" onclick="act('lift_down')">리프트 내림</button>
     <button class="act danger" onclick="act('lift_stop')">리프트 정지</button>
   </div>
   <div class="actmsg">리프트: <span id="liftState">-</span></div>
   <div class="slider" style="margin-top:10px"><label>주행속도</label>
     <input type="range" id="navSpd" min="0.10" max="1.00" step="0.05" value="0.25"
            oninput="navSpeedShow(+this.value)" onchange="navSpeedSet(+this.value)">
     <span class="val" id="navSpdV">0.25</span></div>
   <div id="navSpdMsg" class="actmsg"></div>
   <button class="act" id="btnCart" onclick="toggleCart()">렉 인식 켜기</button>
   <div id="actMsg" class="actmsg"></div>

   <div id="joySlotDash"></div>
   <div id="joyBox" class="joybox">
     <canvas id="joy" width="260" height="260"></canvas>
     <div class="joyhint" id="joyHint"></div>
     <div class="slider"><label>최고속</label>
      <input type="range" id="vLin" min="0.05" max="0.6" value="0.20" step="0.05">
      <span class="val" id="vLinV">0.20</span></div>
     <div class="slider"><label>회전</label>
      <input type="range" id="vAng" min="0.1" max="1.5" value="0.8" step="0.1">
      <span class="val" id="vAngV">0.80</span></div>
   </div>
  </section>
  <section id="mvPanel"><h2>지점 이동</h2>
   <div class="seg" id="mvTabs">
     <button id="mvTabGo" class="sel" onclick="mvTab('go')">바로 이동</button>
     <button id="mvTabRt" onclick="mvTab('route')">왕복</button>
   </div>
   <div id="mvGo">
     <div class="mvgrid" id="mvPts"></div>
     <p class="note" id="mvGoNote">누르면 그 지점으로 바로 갑니다. 자동 모드에서 동작합니다.</p>
   </div>
   <div id="mvRoute" style="display:none">
     <div class="mvrow"><label class="k" for="rtA">A</label><select id="rtA"></select></div>
     <div class="mvrow"><label class="k" for="rtB">B</label><select id="rtB"></select></div>
     <div class="mvrow"><label class="k" for="rtN">반복</label>
       <div class="inl"><input type="number" id="rtN" min="1" max="9999" value="1">
         <label class="tg"><input type="checkbox" id="rtInf" onchange="$('rtN').disabled=this.checked"> 무한</label></div></div>
     <div class="mvrow"><label class="k" for="rtDw">대기</label>
       <div class="inl"><input type="number" id="rtDw" min="0" max="600" value="0">
         <span class="note" style="margin:0">초 (각 지점 도착 후)</span></div></div>
     <div class="actrow g2" style="margin-top:11px">
       <button class="act primary" onclick="routeStart()">왕복 시작</button>
       <button class="act danger" onclick="routeStop()">왕복 중지</button>
     </div>
     <p class="note">1회 = A → B → A. 시작하면 먼저 A 로 갑니다. [정지]·[비상정지]·수동
      전환이나 다른 이동 명령이 들어오면 멈춥니다. 이동이 실패하면 한 번 다시 시도합니다.</p>
   </div>
   <div id="mvMsg" class="actmsg"></div>
   <div class="mvstate" id="mvState"></div>
  </section>
  <section id="cartPanel" style="display:none">
   <h2>렉 인식</h2>
   <div id="cartMsgD" class="actmsg"></div>
   <table id="tCartD"></table>
   <p class="note">라이다로 다리 네 개를 찾아 규격에 맞는 사각형인지 보고,
    카메라로 그 자리에 기둥이 실제로 서 있는지 확인합니다.
    가까운 다리는 카메라 화각을 벗어나 안 보일 수 있습니다.</p>
  </section>
  <section><h2>위치</h2><table id="tPose"></table></section>
  <section><h2>모터 요약</h2><table id="tMotorS"></table></section></div>`,

motor:`<section class="full"><h2>모터 축 상태</h2><table id="tMotor"></table>
  <p class="note">L2DB4830 듀얼 서보 드라이버. CAN 500kbit/s, CANopen SDO.
   축1 = CAN 노드 1, 축2 = CAN 노드 2. 엔코더 4096 counts/회전, 직결.</p></section>`,

battery:`<section class="full"><h2>배터리 상태</h2><div id="batNow">
   <div class="note">배터리 값을 기다리는 중…</div></div></section>
 <section class="full">
  <div class="row" style="margin-bottom:12px">
   <h2 style="margin:0">이력</h2><span class="sp"></span>
   <div class="seg" id="batRange" style="width:330px;max-width:100%">
    <button data-h="1" onclick="batRange(1)">1시간</button>
    <button data-h="6" onclick="batRange(6)" class="sel">6시간</button>
    <button data-h="24" onclick="batRange(24)">24시간</button>
    <button data-h="72" onclick="batRange(72)">3일</button></div></div>
  <div class="bcharts">
   <div class="bchart"><h3>잔량 (%)</h3><div class="bwrap"><canvas id="bcSoc"></canvas><div class="btip"></div></div></div>
   <div class="bchart"><h3>전압 (V)</h3><div class="bwrap"><canvas id="bcV"></canvas><div class="btip"></div></div></div>
   <div class="bchart"><h3>전류 (A, + 충전 / − 사용)</h3><div class="bwrap"><canvas id="bcI"></canvas><div class="btip"></div></div></div>
   <div class="bchart"><h3>온도 (℃)</h3><div class="bwrap"><canvas id="bcT"></canvas><div class="btip"></div></div></div>
  </div>
  <p class="note" id="batHistNote"></p>
  <details class="btab"><summary>표로 보기</summary><table id="tBatHist"></table></details>
 </section>
 <section><h2>경고 기준 (제조사 통신규약)</h2><table id="tBatLim"></table>
  <p class="note">이 값을 넘으면 배터리가 상태 경고를 보냅니다. 사용 시 과전류는 모델마다 다른 참고값입니다.</p></section>
 <section><h2>배터리 정보</h2><table id="tBatInfo"></table></section>`,

lidar:`<section class="full"><h2>LDS-50C-E-12 실시간 스캔</h2>
  <div class="actrow g2" style="max-width:420px">
    <button class="act" onclick="devAct('/devices/lidar/restart','라이다')">스캔 재시작</button>
  </div>
  <div id="devMsgL" class="actmsg"></div>
  <canvas id="cvLidar" width="900" height="640"></canvas>
  <div class="slider"><label>범위</label>
   <input type="range" id="rng" min="1" max="30" value="8" step="1">
   <span class="val" id="rngV">8 m</span></div>
  <table id="tScan"></table></section>`,

camera:`<section class="full"><h2>카메라 — 정면 D435i · 아래 D435F</h2>
  <div class="actrow g2" style="max-width:420px">
    <button class="act" onclick="devAct('/devices/camera/reset','카메라')">하드웨어 리셋</button>
  </div>
  <div id="devMsgC" class="actmsg"></div>
  <div class="camrow">
    <div class="camcell"><div class="camlbl">정면 카메라 (1.0m, 수평) — 렉 인식</div>
      <div class="camwrap"><img id="imgCam" class="cam" alt="정면 카메라">
        <canvas id="cvCam" class="camov"></canvas></div></div>
    <div class="camcell"><div class="camlbl">아래 카메라 (0.96m, 65° 아래)</div>
      <div class="camwrap"><img id="imgCamDn2" class="cam" alt="아래 카메라"></div></div>
  </div>
  <div id="camCartMsg" class="actmsg"></div>
  <table id="tCam"></table>
  <p class="note">뎁스 스트림은 아직 API 에 없습니다. 필요하면
   <code>/camera/camera/depth/image_rect_raw</code> 를 같은 방식으로 추가할 수 있습니다.</p></section>`,

maps:`<section>
  <h2>저장된 맵</h2>
  <table id="tMaps"></table>
  <div id="mapsMsg" class="actmsg"></div>
  <p class="note">목록을 <b>클릭</b>하면 오른쪽에 미리보기만 뜹니다(선택).
   실제로 로봇이 그 지도를 쓰게 하려면 <b>이 지도 적용</b>을 눌러야 합니다.
   적용하면 포즈그래프가 SLAM 에 올라가 이어서 매핑하거나 그 위에서
   위치를 잡을 수 있습니다.</p></section>
 <section>
  <h2>미리보기 <span id="prevName" style="color:var(--dim);font-weight:400"></span></h2>
  <div class="prevbox">
    <img id="mapsPrev" alt="preview">
    <span class="empty" id="prevEmpty">지도를 선택하세요</span>
  </div>
  <table id="tMapDetail" style="margin-top:10px"></table>
  <div class="slider" style="grid-template-columns:44px 1fr 58px;margin-top:10px">
   <label>메모</label>
   <input id="mapNote" placeholder="이 지도에 대한 메모"
     style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
     border-radius:8px;padding:8px 10px;font-size:13px">
   <button class="act" style="margin:0;padding:9px 0" onclick="mapNote()">저장</button>
  </div>
  <div class="row" style="margin-top:12px">
    <button class="act primary" id="btnApply" style="margin:0;flex:1"
      onclick="mapLoad()">이 지도 적용</button>
    <button class="act danger" style="margin:0;flex:1" onclick="mapDelete()">삭제</button>
  </div></section>`,

network:`<section>
  <h2>연결 방식</h2>
  <p class="note">무선 한 장으로 <b>둘 중 하나만</b> 씁니다.
   공유기가 없는 곳에서는 <b>핫스팟</b>으로 로봇에 직접 붙고,
   WiFi 에 붙이면 핫스팟은 꺼집니다.</p>
  <div class="modeseg">
    <button id="nwTogAp"   onclick="nwSetMode('ap')">핫스팟</button>
    <button id="nwTogWifi" onclick="nwSetMode('wifi')">WiFi</button>
  </div>
  <div id="nwModeMsg" class="actmsg"></div>
  <table id="nwApInfo" style="margin-top:8px"></table>
  <div class="slider" style="grid-template-columns:76px 1fr 64px;margin-top:8px">
    <label>핫스팟 이름</label>
    <input id="nwApSsid" placeholder="UND_AMR"
      style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
      border-radius:8px;padding:8px 10px;font-size:13px"><span></span>
  </div>
  <div class="slider" style="grid-template-columns:76px 1fr 64px">
    <label>핫스팟 암호</label>
    <input id="nwApPw" type="password" placeholder="8자 이상"
      style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
      border-radius:8px;padding:8px 10px;font-size:13px">
    <label class="tg" style="font-size:12px"><input type="checkbox"
      onchange="$('nwApPw').type=this.checked?'text':'password'"> 보기</label>
  </div>
  <button class="act" onclick="nwApSave()">핫스팟 설정 저장</button>
  </section>
 <section>
  <h2>WiFi</h2>
  <p class="note">여기서 바꾸면 <b>지금 보고 있는 이 연결이 끊깁니다.</b>
   적용 뒤에는 새 주소로 다시 들어와야 합니다.
   붙지 못하면 <b>원래 연결로 자동 복구</b>되니 로봇이 고립되지는 않습니다.</p>
  <button class="act" id="nwScan" onclick="nwList(true)">주변 WiFi 다시 찾기</button>
  <table id="nwAps"></table>
  <div id="nwPick" style="display:none;margin-top:12px">
    <div class="slider" style="grid-template-columns:76px 1fr 64px">
      <label>SSID</label>
      <input id="nwSsid" readonly
        style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
        border-radius:8px;padding:8px 10px;font-size:13px">
      <span id="nwSec" class="dim-txt" style="font-size:12px"></span>
    </div>
    <div class="slider" style="grid-template-columns:76px 1fr 64px">
      <label>암호</label>
      <input id="nwPw" type="password" placeholder="WiFi 암호"
        style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
        border-radius:8px;padding:8px 10px;font-size:13px">
      <label class="tg" style="font-size:12px"><input type="checkbox"
        onchange="$('nwPw').type=this.checked?'text':'password'"> 보기</label>
    </div>
    <div class="modeseg" style="margin-top:8px">
      <button id="nwMAuto" onclick="nwMode('auto')">자동 (DHCP)</button>
      <button id="nwMMan"  onclick="nwMode('manual')">고정 IP</button>
    </div>
    <div id="nwManual" style="display:none">
      <div class="slider" style="grid-template-columns:76px 1fr 64px">
        <label>IP 주소</label><input id="nwIp" placeholder="192.168.30.50"
          style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
          border-radius:8px;padding:8px 10px;font-size:13px">
        <span class="dim-txt" style="font-size:12px">/ <input id="nwPrefix"
          value="24" style="width:34px;background:#0d0f14;color:var(--fg);
          border:1px solid var(--line);border-radius:6px;padding:4px;font-size:12px"></span>
      </div>
      <div class="slider" style="grid-template-columns:76px 1fr 64px">
        <label>게이트웨이</label><input id="nwGw" placeholder="192.168.30.1"
          style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
          border-radius:8px;padding:8px 10px;font-size:13px"><span></span>
      </div>
      <div class="slider" style="grid-template-columns:76px 1fr 64px">
        <label>DNS</label><input id="nwDns" placeholder="8.8.8.8, 1.1.1.1"
          style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
          border-radius:8px;padding:8px 10px;font-size:13px"><span></span>
      </div>
    </div>
    <button class="act primary" id="nwGo" onclick="nwConnect()">이 WiFi 로 접속</button>
  </div>
  <div id="nwMsg" class="actmsg"></div></section>
 <div style="display:grid;gap:14px;align-content:start">
  <section>
   <h2>지금 연결</h2>
   <table id="nwNow"></table>
   <div id="nwHint" class="actmsg"></div>
  </section>
  <section>
   <h2>주의</h2>
   <p class="note"><b>고정 IP</b> 를 쓸 때는 게이트웨이가 같은 대역이어야
    합니다. 틀리면 붙긴 해도 밖에서 못 들어옵니다 — 그 경우 자동으로
    DHCP 로 되돌립니다.<br><br>
    <b>유선(라이다)</b> 회선은 여기서 건드리지 마세요. 라이다 전용 대역
    (192.168.158.x) 이라 바꾸면 라이다가 끊깁니다.<br><br>
    바꾼 뒤 밖에서 접속이 안 되면 방화벽부터 보세요 —
    <code>sudo ufw allow 8090/tcp</code></p>
  </section>
 </div>`,

cartcal:`<section>
  <h2>새 대차 규격 재기</h2>
  <p class="note">대차를 <b>로봇 앞 1~2 m</b> 에, 주변에 다른 물건 없이 홀로 두세요.
   등록된 규격과 대조하지 않고 <b>가장 직사각형에 가까운 다리 네 개</b>를 찾으므로
   처음 보는 대차도 잽니다. 로봇은 움직이지 않습니다.</p>
  <div class="slider" style="grid-template-columns:76px 1fr 64px">
    <label>이름</label>
    <input id="ccName" placeholder="예: S400"
      style="background:#0d0f14;color:var(--fg);border:1px solid var(--line);
      border-radius:8px;padding:8px 10px;font-size:13px">
    <span class="dim-txt" style="font-size:12px">영문·숫자</span>
  </div>
  <div class="slider" style="grid-template-columns:76px 1fr 64px">
    <label>재는 시간</label>
    <input id="ccSec" type="range" min="3" max="15" step="1" value="6">
    <span id="ccSecV">6 초</span>
  </div>
  <label class="tg" style="margin:6px 2px 0"><input type="checkbox" id="ccFront" checked>
    로봇 앞쪽만 본다 (뒤쪽 벽 모서리가 잡히는 것을 막습니다)</label>
  <button class="act primary" id="ccGo" onclick="ccMeasure()">재기</button>
  <div id="ccMsg" class="actmsg"></div>
  <table id="ccOut" style="margin-top:10px"></table>
  <div id="ccReg" style="display:none;margin-top:12px">
    <button class="act primary" onclick="ccRegister()">이 규격으로 등록</button>
    <p class="note">등록하면 <code>config/carts.yaml</code> 에 써넣고 검출기를
     다시 띄웁니다. 소스와 설치본 둘 다 고치므로 다시 빌드해도 남습니다.</p>
  </div></section>
 <div style="display:grid;gap:14px;align-content:start">
  <section>
   <h2>등록된 대차</h2>
   <table id="ccList"></table>
   <div id="ccFile" class="actmsg"></div>
  </section>
  <section>
   <h2>왜 실측해서 넣나</h2>
   <p class="note">여기 들어가는 값은 줄자로 잰 <b>외곽 치수가 아니라</b>
    라이다가 보는 <b>다리 중심 간 거리</b>입니다.<br><br>
    2026-08-28 에 도면값 765×765 를 그대로 넣었더니 실측 707×688 과
    95 mm 어긋나, 점수가 문턱에 걸쳐 인식률이 <b>11 %</b> 로 깜빡였습니다.
    실측값으로 바꾸니 100 % 가 됐습니다.<br><br>
    <b>흔들림</b>이 20 mm 를 넘으면 다리 하나가 끊겼다 붙었다 하는 상태입니다.
    그대로 등록하면 그 대차는 계속 깜빡이니, 대차를 조금 옮겨 다시 재세요.</p>
  </section>
 </div>`,

mapping:`<section class="mapcard">
  <div class="maphead">
    <h2 style="margin:0" id="mapTitle">매핑</h2>
    <span class="sp"></span>
    <label class="tg"><input type="checkbox" id="tgScan2" checked> 라이다</label>
    <label class="tg"><input type="checkbox" id="tgFollow2" checked> 로봇 추적</label>
    <label class="tg"><input type="checkbox" id="tgTrail2" checked> 이동 궤적</label>
    <label class="tg"><input type="checkbox" id="tgGrid2" checked> 격자</label>
    <button id="btnEdit" onclick="editOpen()">지도 수정</button>
    <button onclick="zoomBy(1.25)">＋</button>
    <button onclick="zoomBy(0.8)">－</button>
    <button onclick="resetView()">맞춤</button>
  </div>
  <div class="relocbar" id="jobBar2">
    <b style="font-size:13px">렉 진입</b>
    <span class="rbmsg" id="jbMsg2"></span>
    <span class="sp"></span>
    <button class="rb" onclick="poiJobCancel()" style="color:var(--bad)">중단</button>
  </div>
  <div class="relocbar" id="editBar">
    <b style="font-size:13px">지도 수정</b>
    <button class="rb sel" id="ebErase" onclick="editOp('erase')">지우개</button>
    <button class="rb" id="ebWall" onclick="editOp('wall')">가상벽</button>
    <button class="rb" id="ebZone" onclick="editOp('zone')">금지구역</button>
    <button class="rb" id="ebDone" onclick="editShapeDone()" style="display:none">완료</button>
    <label class="tg" style="gap:6px">굵기
      <select id="ebSize" onchange="editBrush(this.value)"
        style="background:#1b2130;color:var(--fg);border:1px solid var(--line);
        border-radius:8px;padding:6px 8px;font-size:13px">
        <option value="0.10">10 cm</option>
        <option value="0.25" selected>25 cm</option>
        <option value="0.50">50 cm</option>
      </select></label>
    <button class="rb" onclick="editUndo()">되돌리기</button>
    <button class="rb" onclick="editReset()">초기화</button>
    <span class="sp"></span>
    <button class="rb go" onclick="editConfirm()">확인</button>
    <button class="rb" onclick="editClose()">취소</button>
    <span class="rbmsg" id="ebMsg"></span>
  </div>
  <div class="relocbar" id="relocBar2">
    <b style="font-size:13px">위치 재조정</b>
    <button class="rb" id="rbMove2" onclick="relocMode('move')">위치</button>
    <button class="rb" id="rbRot2"  onclick="relocMode('rot')">회전</button>
    <button class="rb" onclick="relocAuto()">자동</button>
    <span class="nud" title="미세 이동 (로봇 기준)">
      <button class="rb" onclick="relocNudge(1,0,0)" title="앞">▲</button>
      <button class="rb" onclick="relocNudge(-1,0,0)" title="뒤">▼</button>
      <button class="rb" onclick="relocNudge(0,1,0)" title="왼쪽">◀</button>
      <button class="rb" onclick="relocNudge(0,-1,0)" title="오른쪽">▶</button>
      <button class="rb" onclick="relocNudge(0,0,1)" title="왼쪽으로 돌기">⟲</button>
      <button class="rb" onclick="relocNudge(0,0,-1)" title="오른쪽으로 돌기">⟳</button>
      <button class="rb" data-big="0" onclick="relocStep(this)" title="한 번에 움직이는 양">1cm·0.5°</button>
    </span>
    <button class="rb" onclick="relocRelease()" title="수동 고정을 풀고 라이다로 다시 맞춥니다">보정 켜기</button>
    <span class="sp"></span>
    <button class="rb go" onclick="relocConfirm()">확인</button>
    <button class="rb" onclick="relocCancel()">취소</button>
    <span class="rbmsg" id="rbMsg2"></span>
  </div>
  <canvas id="cvMap2" width="960" height="620"></canvas>
  <div class="maphint" id="mapHint2"></div></section>
 <div style="display:grid;gap:14px;align-content:start">
  <section>
   <h2>매핑 제어</h2>
   <button class="act primary" id="btnStart" onclick="mapStart()">새 매핑 시작</button>
   <button class="act" id="btnResume" onclick="mapResume()">이어서 매핑</button>
   <div id="resumeMsg" class="actmsg"></div>
   <div id="activeMap" class="actmsg"></div>
   <div class="actrow g2">
     <button class="act" id="btnPause" onclick="mapPause()">일시정지</button>
     <button class="act danger" id="btnStop" onclick="mapStop()">매핑 종료</button>
   </div>
   <button class="act" onclick="mapClear()">보정 초기화</button>
   <div id="mapMsg" class="actmsg"></div>
   <p class="note">지도는 로봇이 움직인 만큼 넓어집니다. 아래 조이스틱으로
    천천히 주행하면서 화면이 채워지는 것을 보세요. 다 돌면 <b>매핑 종료</b>를
    누르면 저장할지 묻고, 저장하면 그 지도가 곧바로 메인 맵이 됩니다.
    저장한 지도는 <b>맵</b> 화면에서 관리합니다.</p>
   <p class="note"><b>이어서 매핑</b>은 이미 만들어 둔 지도에 덧그립니다.
    로봇이 지도 어디에 있는지 알아야 그 자리에서 이어 그릴 수 있으므로,
    지도를 적용하고 위치를 잡은 뒤에 눌립니다. 위치를 모르는 채로 이어
    그리면 엉뚱한 곳에 새 영역이 생겨 지도가 망가집니다.</p>
  </section>
  <section>
   <h2>주행</h2>
   <div class="modeseg">
     <button id="mapMAuto" onclick="setMode('auto')">자동</button>
     <button id="mapMMan"  onclick="setMode('manual')">수동</button>
   </div>
   <button class="act primary" onclick="act('enable')">모터 사용 (WAKE UP)</button>
   <div class="actrow g2">
     <button class="act danger" onclick="act('stop')">정지</button>
     <button class="act" onclick="act('release')">축 해제</button>
   </div>
   <div id="joySlotMapping"></div>
   <p class="note">수동 모드에서만 조이스틱이 동작합니다. 손을 떼면 즉시 멈추고,
    통신이 끊겨도 0.5초 안에 정지합니다. 화면을 옮겨도 조이스틱은 따라옵니다.</p>
  </section>
  <section><h2>POI</h2>
   <div id="poiSee" class="note" style="margin:0 0 9px"></div>
   <div class="row" style="gap:7px;margin-bottom:9px">
     <input id="poiName" placeholder="이름 (예: 렉1)" maxlength="40"
       style="flex:1;min-width:90px;background:#1b2130;color:var(--fg);
       border:1px solid var(--line);border-radius:8px;padding:7px 10px;font-size:13px">
     <select id="poiKind" onchange="poiKindChange()"
       style="background:#1b2130;color:var(--fg);border:1px solid var(--line);
       border-radius:8px;padding:7px 8px;font-size:13px">
       <option value="cart">대차</option>
       <option value="point">지점</option>
       <option value="charger">충전소</option>
     </select>
   </div>
   <div class="row" id="poiCartRow" style="gap:7px;margin-bottom:9px">
     <label class="tg" style="gap:6px">대차 종류
       <select id="poiCart"
         style="background:#1b2130;color:var(--fg);border:1px solid var(--line);
         border-radius:8px;padding:7px 8px;font-size:13px"></select></label>
   </div>
   <div class="slider" id="poiSoRow"><label>접근거리</label>
     <input type="range" id="poiSo" min="1.2" max="2.5" step="0.1" value="1.8"
            oninput="$('poiSoV').textContent=(+this.value).toFixed(1)">
     <span class="val" id="poiSoV">1.8</span></div>
   <button class="act primary" onclick="poiSave()">지금 자리로 등록</button>
   <div id="poiMsg" class="actmsg"></div>
   <table id="tPoi"></table>
   <p class="note" id="poiHint"></p>
  </section>
  <section><h2>작성 상태</h2><table id="tMapping"></table></section>
  <section><h2>주행 기록</h2>
   <table id="tRec"></table>
   <p class="note">매핑하는 동안 라이다·TF·오도메트리를 통째로 녹화합니다.
    저장하면 지도와 같은 이름의 <code>.bag</code> 으로 함께 남아, 나중에
    파라미터를 바꿔 다시 돌려보거나 문제를 되짚을 때 씁니다.
    저장하지 않고 종료하면 그 녹화는 버려집니다.</p>
  </section>
 </div>`,

nodes:`<section class="full"><h2>ROS 노드</h2><pre id="pNodes"></pre></section>`,
topics:`<section class="full"><h2>ROS 토픽</h2><table id="tTopics"></table></section>`,

log:`<section class="full"><h2>로그 (/rosout)</h2>
  <div class="row" style="margin-bottom:9px">
   <button onclick="logLevel('')">전체</button>
   <button onclick="logLevel('INFO')">INFO</button>
   <button onclick="logLevel('WARN')">WARN</button>
   <button onclick="logLevel('ERROR')">ERROR</button></div>
  <pre id="pLog"></pre></section>`,

services:`<section class="full"><h2>서비스</h2>
  <div class="row">
   <button onclick="svc('/services/stop')">즉시 정지</button>
   <button onclick="svc('/services/clear_errors')">모터 알람 클리어</button></div>
  <p class="note">알람 클리어는 아직 미구현입니다 — base_node 에 시리얼 명령
   서비스를 추가해야 합니다. 지금은 STM32 콘솔에서 <code>a</code> 를 입력하세요.</p>
  <table id="tSvc" style="margin-top:12px"></table></section>`,

device:`<section class="full"><h2>장비 정보</h2><pre id="pDev"></pre></section>`,
};

// ---------------------------------------------------------------- 상태
let cur='dash', scan=null, pose=null, status=null, batt=null, dev=null;
let estop=false, mode='auto', driving=null, logFilter='', camOn=false;

// 맵 뷰 상태 (월드 m ↔ 화면 px)
// 차체 치수 [m]. urdf/amr.urdf.xacro 실측값과 같은 값을 쓴다.
//   길이 0.80, 폭 0.27, 구동축(로봇 좌표 원점)은 앞끝에서 0.38
const ROBOT={len:0.80, wid:0.27, front:0.38};
let mapMeta=null, mapImg=null, mapSeq=-1, mapPose=null, trail=[];
// nav2 가 낸 경로. 로봇이 '어디로 갈 작정인지' 를 보여 준다.
// 지나온 자취(trail)와 구분되게 다른 색으로 그린다.
let navPath=[];
// 지금 궤적이 어느 지도 위의 것인지. 지도가 바뀌면 궤적을 버린다.
let trailMap=null;
// 렉 인식은 화면마다 따로 두지 않고 상태 하나로 관리한다.
// 화면별 토글을 각각 두었더니 어느 쪽을 보느냐에 따라 동작이 갈렸다.
let cartOn=false;
let resumedFrom=null;   // 이어서 매핑을 시작한 지도 이름 (서버가 알려준다)
let carts=null;   // 최근 렉 인식 결과
let cartsErr=null;
let view={cx:0, cy:0, ppm:40, dragging:false, lx:0, ly:0};   // ppm = px per meter
let viewRestored=false;   // 저장된 화면 위치를 되살렸는가

// 화면 설정을 브라우저에 저장한다. 서버를 다시 띄우거나 새 창을 열 때마다
// 격자·추적 토글과 지도 위치가 처음으로 돌아가 버려서, 볼 때마다 다시
// 맞춰야 했다. 사람이 맞춰 둔 상태는 그대로 두는 게 맞다.
const UI_KEY='und_amr_ui_v1';
const UI_TOGGLES=['tgScan','tgFollow','tgGrid','tgScan2','tgFollow2',
                  'tgTrail2','tgGrid2'];
function saveUI(){
  try{
    const t={};
    for(const id of UI_TOGGLES){ const e=$(id); if(e) t[id]=e.checked; }
    localStorage.setItem(UI_KEY, JSON.stringify({
      t, cartOn, view:{cx:view.cx, cy:view.cy, ppm:view.ppm}}));
  }catch(e){}
}
function loadUI(){
  try{
    const d=JSON.parse(localStorage.getItem(UI_KEY)||'{}');
    if(d.t) for(const id of UI_TOGGLES){
      const e=$(id); if(e && d.t[id]!=null) e.checked=!!d.t[id];
    }
    if(d.view && d.view.ppm>0){
      view.cx=d.view.cx; view.cy=d.view.cy; view.ppm=d.view.ppm;
      viewRestored=true;
    }
    if(d.cartOn){ cartOn=true; syncCartBtn(); }
  }catch(e){}
}
function syncCartBtn(){
  const b=$('btnCart');
  if(b){ b.textContent = cartOn ? '렉 인식 끄기' : '렉 인식 켜기';
         b.className = 'act' + (cartOn ? ' primary' : ''); }
}

async function api(p,o){ const r=await fetch(p,o); return r.json(); }
function chip(id,t,c){ const e=document.getElementById(id); if(!e)return;
  e.textContent=t; e.className='chip'+(c?' '+c:''); }
function row(k,v){ return '<tr><td>'+k+'</td><td>'+v+'</td></tr>'; }
function fmtDur(sec){
  if(sec==null) return '-';
  const s=Math.round(sec), m=Math.floor(s/60);
  return m ? m+'분 '+(s%60)+'초' : s+'초';
}
function fmtMB(b){ return b==null?'-':(b/1048576).toFixed(1)+' MB'; }
function $(id){ return document.getElementById(id); }

// ---------------------------------------------------------------- 지점 이동
// 대시보드에서 등록한 '지점' 으로 바로 가거나 A/B 를 오간다. 왕복은
// api_node 가 돌린다 (브라우저를 닫아도 계속). 대차 POI 는 좌표가 렉
// 한가운데라 여기서는 뺀다 — 대차는 [도킹] 으로.
let mvCur='go', mvNames=null, mvRoute=null, mvNav=null;
function mvPoints(){ return poiList.filter(q=>q.kind!=='cart' && q.kind!=='charger'); }
// 2026-11-18 왕복 끝점에는 충전소도 된다 (충전소 쪽은 도킹으로 도착)
function mvRoutePts(){ return poiList.filter(q=>q.kind!=='cart'); }
function mvTab(t){
  mvCur=t;
  $('mvTabGo').className=t==='go'?'sel':''; $('mvTabRt').className=t==='route'?'sel':'';
  $('mvGo').style.display=t==='go'?'':'none'; $('mvRoute').style.display=t==='route'?'':'none';
}
function mvSay(m,bad){ const e=$('mvMsg'); if(!e) return;
  e.textContent=m||''; e.style.color=bad?'var(--bad)':'var(--dim)'; }
function mvRenderPts(){
  const pts=mvPoints(), rpts=mvRoutePts(), names=rpts.map(q=>q.name+':'+q.kind).join('\n');
  if(names===mvNames) return;
  mvNames=names;
  const g=$('mvPts'); if(!g) return;
  // 이름은 사용자가 넣은 글자라 innerHTML 에 끼우지 않는다 (따옴표가 들어가면 깨진다)
  g.replaceChildren();
  if(!pts.length){
    g.innerHTML='<div class="note" style="grid-column:1/-1;margin:0">등록된 지점이 없습니다. '+
      '<b>매핑</b> 화면 POI 에서 종류를 <b>지점</b>으로 두고 등록하세요.</div>';
  }
  for(const q of pts){
    const b=document.createElement('button');
    b.textContent=q.name; b.title=q.name; b.dataset.n=q.name;
    b.onclick=()=>mvGo(q.name); g.appendChild(b);
  }
  for(const id of ['rtA','rtB']){
    const sel=$(id); if(!sel) continue;
    const keep=sel.value;
    sel.replaceChildren(...rpts.map(q=>new Option(q.kind==='charger'?q.name+' (충전소 도킹)':q.name,q.name)));
    if(rpts.some(q=>q.name===keep)) sel.value=keep;
  }
  if(rpts.length>1 && $('rtA').value===$('rtB').value) $('rtB').selectedIndex=1;
}
async function mvPost(path,body){
  const r=await api(path,{method:'POST',headers:{'Content-Type':'application/json'},
    body:JSON.stringify(body||{})});
  mvSay(r.message,!r.ok); mvPoll(); return r;
}
async function mvGo(name){
  if(mvRoute&&mvRoute.busy){
    if(!confirm('왕복 중입니다. 멈추고 '+name+' 로 갈까요?')) return;
    try{ await api('/route/cancel',{method:'POST',headers:{'Content-Type':'application/json'},body:'{}'}); }
    catch(e){ mvSay('왕복을 멈추지 못했습니다',true); return; }
  }
  try{ await mvPost('/pois/goto',{name:name}); }catch(e){ mvSay('요청 실패',true); }
}
async function routeStart(){
  const a=$('rtA').value, b=$('rtB').value;
  if(!a||!b){ mvSay('지점이 두 개 이상 있어야 합니다',true); return; }
  if(a===b){ mvSay('A 와 B 를 다르게 고르세요',true); return; }
  const inf=$('rtInf').checked, n=Math.round(+$('rtN').value);
  if(!inf && !(n>=1)){ mvSay('반복 횟수는 1 이상이어야 합니다',true); return; }
  try{ await mvPost('/route/start',{a:a,b:b,repeat:inf?0:n,dwell:+$('rtDw').value||0}); }
  catch(e){ mvSay('요청 실패',true); }
}
async function routeStop(){ try{ await mvPost('/route/cancel'); }catch(e){ mvSay('요청 실패',true); } }
function mvTargetName(t){
  if(!t) return null;
  const q=poiList.find(p=>Math.hypot(p.x-t.x,p.y-t.y)<0.02);
  return q?q.name:null;
}
let cdock=null;
const CD_TXT={moving:'대기 자리로 이동',aligning:'충전소 인식',backing:'후진 진입',
  done:'완료',failed:'실패',canceled:'멈춤'};
async function mvPoll(){
  if(cur!=='dash') return;
  try{ [mvRoute,mvNav,cdock]=await Promise.all([api('/route/status'),api('/nav/status'),
       api('/charger/status').catch(()=>null)]); }
  catch(e){ return; }
  const el=$('mvState'); if(!el) return;
  const R=mvRoute, N=mvNav, tn=mvTargetName(N.target);
  const RT={going:'이동 중',docking:'충전소 도킹 중',waiting:'대기 중',retry:'다시 시도 대기',lifting:'리프트 동작 중',done:'완료',
            failed:'실패',canceled:'멈춤'};
  let h='';
  const CDk=cdock;
  if(CDk && CDk.state && CDk.state!=='idle' && (CDk.busy || Date.now()/1000-(CDk.started||0)<1800)){
    const cls=CDk.state==='failed'?'bad-txt':CDk.busy?'ok-txt':'';
    h='<b class="'+cls+'">충전소 도킹 '+(CD_TXT[CDk.state]||CDk.state)+'</b> · '+esc(CDk.poi||'')
      +'<br><span class="note" style="margin:0">'+esc(CDk.message||'')+'</span>'
      +(CDk.busy?' <button class="rb" onclick="cdockCancel()">중단</button>':'');
  }else if(R && R.state && R.state!=='idle' && (R.busy || Date.now()/1000-(R.started||0)<3600)){
    const rep=R.repeat? (R.lap+' / '+R.repeat+'회') : (R.lap+'회 (무한)');
    const cls=R.state==='failed'?'bad-txt':R.busy?'ok-txt':'';
    h='<b class="'+cls+'">왕복 '+(RT[R.state]||R.state)+'</b> · '+esc(R.a)+' ↔ '+esc(R.b)+
      ' · '+rep+'<br><span class="note" style="margin:0">'+esc(R.message||'')+
      (R.remain_s!=null?' ('+Math.ceil(R.remain_s)+'초)':'')+'</span>';
  }else if(N && N.state && N.state!=='idle'){
    const NT={sending:'보내는 중',active:'이동 중',done:'도착',failed:'실패',canceled:'취소됨'};
    const cls=N.state==='failed'?'bad-txt':N.state==='active'||N.state==='sending'?'ok-txt':'';
    h='<b class="'+cls+'">'+(tn?esc(tn)+' ':'')+(NT[N.state]||N.state)+'</b>'+
      (N.message?'<br><span class="note" style="margin:0">'+esc(N.message)+'</span>':'');
  }
  el.innerHTML=h; el.className='mvstate'+(h?' on':'');
  const moving=N&&(N.state==='active'||N.state==='sending');
  document.querySelectorAll('#mvPts button[data-n]').forEach(b=>
    b.className=(moving&&b.dataset.n===tn)?'sel':'');
}
setInterval(mvPoll,700);

// ---------------------------------------------------------------- 배터리
// 타보스 BMS (CAN2 -> STM32 -> base_node -> api_node). 값이 끊기면 모터
// 드라이버 모선 전압만 보여 준다.
const BAT_CH=[
  {id:'bcSoc',k:'soc', name:'잔량',u:'%', fix:[0,100],area:true,d:0},
  {id:'bcV',  k:'v',   name:'전압',u:' V',pad:0.2,d:2},
  {id:'bcI',  k:'i',   name:'전류',u:' A',pad:0.5,zero:true,d:2},
  {id:'bcT',  k:'temp',name:'온도',u:' ℃',pad:1.0,d:1},
];
let batH=6, batHist=null, batHover=null;
function socCls(s){ return s<20?'bad':s<40?'warn':'ok'; }
function fmtMin(m){
  if(m==null) return '-';
  const h=Math.floor(m/60); return h? h+'시간 '+(m%60)+'분' : m+'분';
}
function tile(k,v,u){ return '<div class="tile"><div class="k">'+k+'</div><div class="v">'+
  v+(u?'<small>'+u+'</small>':'')+'</div></div>'; }
function renderBattery(b){
  const cb=$('cBatt');
  if(cb){
    if(b.source==='bms'){
      const s=Math.max(0,Math.min(100,b.soc));
      cb.className='chip batchip '+socCls(s);
      cb.innerHTML='<span class="bicon"><i style="width:calc('+s+'% - 2px)"></i></span>'+
        s+'%'+(b.state==='charging'?' · 충전 중':'')+(b.flags&&b.flags.length?' · ⚠':'');
      cb.title='배터리 '+s+'% · '+b.voltage.toFixed(2)+' V · '+b.current.toFixed(2)+' A';
    }else if(b.voltage){
      cb.className='chip'+(b.voltage<20?' bad':b.voltage<22?' warn':'');
      cb.textContent=b.voltage.toFixed(1)+' V';
      cb.title='배터리 BMS 값 없음 — 모터 드라이버 전압';
    }else{ cb.className='chip'; cb.textContent='배터리 -'; cb.title=''; }
  }
  if(cur!=='battery' || !$('batNow')) return;
  if(b.source==='bms'){
    const s=b.soc, st=b.state==='charging'?'충전 중':b.state==='discharging'?'사용 중':'대기';
    const fl=b.flags||[];
    $('batNow').innerHTML=
      '<div class="bathero"><div><div class="bigsoc">'+s+'<small>%</small></div>'+
      '<div class="batsub">'+st+' · '+b.voltage.toFixed(2)+' V · '+
      (b.current>0?'+':b.current<0?'−':'')+Math.abs(b.current).toFixed(2)+' A</div></div>'+
      '<div style="flex:1;min-width:220px"><div class="meter"><i class="'+socCls(s)+
      '" style="width:'+s+'%"></i></div><div class="batsub">남은 용량 '+
      b.remaining_ah.toFixed(2)+' / '+b.design_ah+' Ah · '+b.energy_wh.toFixed(1)+' Wh · '+
      (b.age_s!=null?b.age_s.toFixed(1)+'초 전 갱신':'')+'</div>'+
      '<div class="bflags">'+(fl.length? fl.map(f=>'<span class="chip bad">⚠ '+f+'</span>').join('')
        : '<span class="chip ok">✓ 경고 없음</span>')+'</div></div></div>'+
      '<div class="tiles">'+
      tile('전압',b.voltage.toFixed(2),'V')+
      tile('전류',(b.current>0?'+':b.current<0?'−':'')+Math.abs(b.current).toFixed(2),'A')+
      tile('전력',Math.abs(b.power_w).toFixed(1),'W')+
      tile('온도',b.temperature.toFixed(1),'℃')+
      tile('건강도 (SOH)',b.soh,'%')+
      tile(b.state==='charging'?'완충까지':'사용 가능 시간',
           b.state==='charging'?fmtMin(b.time_to_full_min):
           b.state==='discharging'?fmtMin(b.time_to_empty_min):'-','')+
      tile('사이클',b.cycles,'회')+'</div>';
  }else{
    $('batNow').innerHTML='<div class="note"><span class="bad-txt">배터리 BMS 값이 없습니다</span> — '+
      (b.bms_error||b.error||'-')+'<br>모터 드라이버 모선 전압: <b>'+
      (b.voltage?b.voltage.toFixed(1)+' V':'-')+'</b></div>';
  }
  const L=b.limits||{};
  if($('tBatLim')) $('tBatLim').innerHTML=
    row('과전압','≥ '+(L.over_v??'-')+' V')+row('저전압','≤ '+(L.under_v??'-')+' V')+
    row('충전 시 과전류','≥ '+(L.chg_a??'-')+' A')+row('사용 시 과전류 (참고)','≥ '+(L.dis_a??'-')+' A')+
    row('고온','≥ '+(L.hot_c??'-')+' ℃')+row('저온','≤ '+(L.cold_c??'-')+' ℃');
  if($('tBatInfo')) $('tBatInfo').innerHTML=
    row('모델',b.model||'타보스 LV-25V25AH-DH255COM')+
    row('구성','리튬이온 7S · 설계 용량 '+(b.design_ah||25)+' Ah')+
    row('통신','CAN2 500 kbit/s · ID 0x460 · 1초마다 요청')+
    row('데이터 출처',b.source==='bms'?'<span class="ok-txt">BMS</span>':'<span class="warn-txt">모터 드라이버 (BMS 없음)</span>')+
    row('모터 드라이버 모선 전압',(b.bus_voltage??b.voltage)?(b.bus_voltage??b.voltage).toFixed(1)+' V':'-');
}
async function loadBatHist(){
  try{ batHist=await api('/battery-history?hours='+batH); }catch(e){ return; }
  drawBatCharts(); batTable();
}
function batRange(h){
  batH=h; batHover=null;
  document.querySelectorAll('#batRange button').forEach(b=>b.className=(+b.dataset.h===h?'sel':''));
  loadBatHist();
}
function niceStep(span,n){
  const raw=Math.max(span,1e-6)/n, p=Math.pow(10,Math.floor(Math.log10(raw))), m=raw/p;
  return (m<1.5?1:m<3?2:m<7?5:10)*p;
}
function batTime(t,withDate){
  const d=new Date(t*1000), hh=String(d.getHours()).padStart(2,'0'),
        mm=String(d.getMinutes()).padStart(2,'0');
  return withDate? (d.getMonth()+1)+'/'+d.getDate()+' '+hh+'시' : hh+':'+mm;
}
function drawBatCharts(){
  const H=batHist;
  for(const c of BAT_CH) drawBatChart(c,H);
  const n=$('batHistNote');
  if(n) n.textContent=H&&H.t&&H.t.length
    ? '10초마다 기록해 3일 보관합니다. 그래프의 점 하나는 '+Math.round(H.step_s)+'초 평균입니다. 로봇이 꺼져 있던 구간은 선이 끊깁니다.'
    : '';
}
function drawBatChart(c,H){
  const cv=$(c.id); if(!cv) return;
  const W=cv.clientWidth||600, Hh=170, dpr=window.devicePixelRatio||1;
  cv.width=W*dpr; cv.height=Hh*dpr;
  const g=cv.getContext('2d'); g.setTransform(dpr,0,0,dpr,0,0); g.clearRect(0,0,W,Hh);
  const L=48,R=12,T=10,B=22, pw=W-L-R, ph=Hh-T-B;
  const ts=(H&&H.t)||[], vs=(H&&H[c.k])||[];
  const tEnd=H&&H.now?H.now:Date.now()/1000, t0=tEnd-batH*3600;
  let lo=0,hi=1;
  if(c.fix){ [lo,hi]=c.fix; }
  else if(vs.length){
    lo=Math.min(...vs); hi=Math.max(...vs);
    if(c.zero){ lo=Math.min(lo,0); hi=Math.max(hi,0); }
    const pad=Math.max((hi-lo)*0.15,c.pad); lo-=pad; hi+=pad;
  }
  const step=niceStep(hi-lo,4);
  if(!c.fix){ lo=Math.floor(lo/step)*step; hi=Math.ceil(hi/step)*step; }
  const X=t=>L+(t-t0)/(tEnd-t0)*pw, Y=v=>T+(hi-v)/(hi-lo)*ph;
  const dec=step<0.1?2:step<1?1:0;
  // 기록이 없으면 눈금도 그리지 않는다 (0~1 같은 의미 없는 축이 된다)
  const wrap=cv.parentElement;
  let em=wrap.querySelector('.bempty');
  if(!vs.length){
    if(!em){ em=document.createElement('div'); em.className='bempty'; wrap.appendChild(em); }
    em.textContent='아직 기록이 없습니다 — 10초마다 쌓입니다';
    cv._geo=null; return;
  }
  if(em) em.remove();
  g.font='11px system-ui,-apple-system,"Noto Sans KR",sans-serif'; g.lineWidth=1;
  g.textAlign='right'; g.textBaseline='middle';
  for(let v=lo; v<=hi+step*1e-6; v+=step){
    const y=Math.round(Y(v))+0.5;
    g.strokeStyle=(c.zero&&Math.abs(v)<step*1e-6)?'#46506a':'#2a3140';
    g.beginPath(); g.moveTo(L,y); g.lineTo(W-R,y); g.stroke();
    g.fillStyle='#8f97a8'; g.fillText(v.toFixed(dec),L-7,y);
  }
  const xs=batH<=1?900:batH<=6?3600:batH<=24?4*3600:12*3600;
  const off=-new Date().getTimezoneOffset()*60;
  g.textAlign='center'; g.textBaseline='top'; g.fillStyle='#8f97a8';
  for(let t=Math.ceil((t0+off)/xs)*xs-off; t<=tEnd; t+=xs){
    const x=X(t); if(x<L+14||x>W-R-14) continue;
    g.fillText(batTime(t,batH>24),x,Hh-B+6);
  }
  const gap=Math.max(3*(H.step_s||10),60);
  const segs=[]; let seg=[];
  for(let i=0;i<ts.length;i++){
    if(seg.length && ts[i]-ts[i-1]>gap){ segs.push(seg); seg=[]; }
    seg.push(i);
  }
  if(seg.length) segs.push(seg);
  if(c.area){
    g.fillStyle='rgba(77,163,255,.10)';
    for(const sg of segs){
      g.beginPath(); g.moveTo(X(ts[sg[0]]),Y(lo));
      for(const i of sg) g.lineTo(X(ts[i]),Y(vs[i]));
      g.lineTo(X(ts[sg[sg.length-1]]),Y(lo)); g.closePath(); g.fill();
    }
  }
  g.strokeStyle='#4da3ff'; g.lineWidth=2; g.lineJoin='round'; g.lineCap='round';
  for(const sg of segs){
    g.beginPath();
    sg.forEach((i,j)=>{ const x=X(ts[i]),y=Y(vs[i]); j?g.lineTo(x,y):g.moveTo(x,y); });
    if(sg.length===1){ const x=X(ts[sg[0]]),y=Y(vs[sg[0]]); g.lineTo(x+0.1,y); }
    g.stroke();
  }
  const dot=(i)=>{ const x=X(ts[i]),y=Y(vs[i]);
    g.beginPath(); g.arc(x,y,6,0,7); g.fillStyle='#181b23'; g.fill();
    g.beginPath(); g.arc(x,y,4,0,7); g.fillStyle='#4da3ff'; g.fill(); };
  if(batHover!=null && batHover<ts.length){
    const x=Math.round(X(ts[batHover]))+0.5;
    g.strokeStyle='rgba(143,151,168,.55)'; g.lineWidth=1;
    g.beginPath(); g.moveTo(x,T); g.lineTo(x,T+ph); g.stroke();
    dot(batHover);
  }else dot(ts.length-1);
  cv._geo={X,t0,tEnd,L,pw};
  if(!cv._bound){
    cv._bound=true;
    const move=e=>{
      const H2=batHist, geo=cv._geo; if(!H2||!H2.t||!H2.t.length||!geo) return;
      const r=cv.getBoundingClientRect(), x=e.clientX-r.left;
      const t=geo.t0+(x-geo.L)/geo.pw*(geo.tEnd-geo.t0);
      let a=0,b=H2.t.length-1;
      while(b-a>1){ const m=(a+b)>>1; if(H2.t[m]<t) a=m; else b=m; }
      const i=Math.abs(H2.t[a]-t)<Math.abs(H2.t[b]-t)?a:b;
      if(Math.abs(geo.X(H2.t[i])-x)>40){ leave(); return; }
      batHover=i; drawBatCharts();
      const tip=cv.parentElement.querySelector('.btip');
      tip.innerHTML='<span>'+batTime(H2.t[i],batH>24)+(batH>24?'':'')+'</span><br>'+
        BAT_CH.map(c2=>'<span>'+c2.name+'</span> <b>'+
          (+H2[c2.k][i]).toFixed(c2.d)+c2.u+'</b>').join('<br>');
      tip.style.display='block';
      const tw=tip.offsetWidth, px=geo.X(H2.t[i]);
      tip.style.left=(px+12+tw>r.width? px-12-tw : px+12)+'px';
      tip.style.top='8px';
    };
    const leave=()=>{
      if(batHover==null) return;
      batHover=null; drawBatCharts();
      document.querySelectorAll('.bwrap .btip').forEach(t=>t.style.display='none');
    };
    cv.addEventListener('pointermove',move);
    cv.addEventListener('pointerdown',move);
    cv.addEventListener('pointerleave',leave);
  }
}
function batTable(){
  const el=$('tBatHist'), H=batHist; if(!el) return;
  if(!H||!H.t||!H.t.length){ el.innerHTML=row('기록','없음'); return; }
  const n=H.t.length, k=Math.max(1,Math.ceil(n/48));
  let h='<tr><th>시각</th><th>잔량</th><th>전압</th><th>전류</th><th>온도</th></tr>';
  for(let i=n-1;i>=0;i-=k)
    h+='<tr><td>'+batTime(H.t[i],true).replace('시','')+':'+
      String(new Date(H.t[i]*1000).getMinutes()).padStart(2,'0')+'</td><td>'+
      H.soc[i].toFixed(0)+' %</td><td>'+H.v[i].toFixed(2)+' V</td><td>'+
      H.i[i].toFixed(2)+' A</td><td>'+H.temp[i].toFixed(1)+' ℃</td></tr>';
  el.innerHTML=h;
}
window.addEventListener('resize',()=>{ if(cur==='battery') drawBatCharts(); });
setInterval(()=>{ if(cur==='battery') loadBatHist(); },30000);

// ---------------------------------------------------------------- 네비
function buildNav(){
  let html='', grp=null;
  for(const s of SECTIONS){
    if(s.grp!==grp){ grp=s.grp; html+='<div class="navgrp">'+grp+'</div>'; }
    html+='<div class="nav" id="nav-'+s.id+'" onclick="go(\''+s.id+'\')">'+
          '<span class="dot '+(s.ok?'on':'off')+'" id="dot-'+s.id+'"></span>'+s.name+'</div>';
  }
  $('nav').innerHTML=html;
  let panes='';
  for(const s of SECTIONS){
    panes+='<div class="wrap" id="pane-'+s.id+'">'+
      (s.ok? PANE[s.id] : '<section class="full"><div class="todo"><b>미구현</b>'+
       s.need+'</div></section>')+'</div>';
  }
  $('panes').innerHTML=panes;
}
function go(id, fromHash){
  if(!SECTIONS.some(s=>s.id===id)) id='dash';
  cur=id;
  if(!fromHash && location.hash.slice(1)!==id) location.hash=id;
  for(const s of SECTIONS){
    $('pane-'+s.id).className='wrap'+(s.id===id?' sel':'');
    $('nav-'+s.id).className='nav'+(s.id===id?' sel':'');
  }
  $('hTitle').textContent=(SECTIONS.find(s=>s.id===id)||{}).name||'';
  // 지도 수정은 매핑 화면에서만 한다. 다른 화면으로 가면 닫는다
  // (칠하던 것은 버린다 — 확인을 누르지 않았으므로 반영된 것이 없다)
  if(id!=='mapping' && edit.on){ editResetClose(); editClose(); }
  moveJoy(id);
  if(id==='dash'){ bindMapDrag(); bindJoy(); drawJoy(); refreshMap(); }
  if(id==='camera'){ startCam(); drawCamOverlay('imgCam','cvCam','camCartMsg'); }
  if(id==='nodes') loadNodes();
  if(id==='topics') loadTopics();
  if(id==='maps') loadMaps();
  if(id==='battery'){ renderBattery(batt||{}); loadBatHist(); }
  if(id==='mapping'){ bindMapDrag(); bindJoy(); drawJoy(); loadMapping(); refreshMap(); }
  if(id==='device') loadDev();
  if(id==='cartcal') ccLoad();
  if(id==='network'){ nwNow(); nwList(false); }
  if(id==='services') loadSvc();
  draw();
}

// ---------------------------------------------------------------- 제어
async function setMode(m){
  await api('/chassis/status',{method:'PATCH',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({control_mode:m})});
  refresh();
}
async function toggleEstop(){
  await api('/services/estop',{method:'POST',
    headers:{'Content-Type':'application/json'},body:JSON.stringify({enable:!estop})});
  refresh();
}
function twist(l,a){ fetch('/chassis/twist',{method:'POST',
  headers:{'Content-Type':'application/json'},
  body:JSON.stringify({linear:l,angular:a})}).catch(()=>{}); }
function stopNow(){ fetch('/services/stop',{method:'POST'}).catch(()=>{}); }
async function svc(p){ try{ const r=await api(p,{method:'POST'});
  alert(r.message||JSON.stringify(r)); }catch(e){} }

// ---------------------------------------------------------------- 액션 버튼
async function act(name){
  const msg=$('actMsg');
  const map={stop:'/services/stop', enable:'/services/enable',
             release:'/services/release', clear_errors:'/services/clear_errors',
             lift_up:'/lift/up', lift_down:'/lift/down', lift_stop:'/lift/stop'};
  try{
    const r=await api(map[name],{method:'POST'});
    if(msg){ msg.textContent=r.message||(r.ok?'완료':'실패');
             msg.style.color = r.ok===false ? 'var(--bad)' : 'var(--dim)'; }
  }catch(e){ if(msg){ msg.textContent='요청 실패'; msg.style.color='var(--bad)'; } }
  refresh();
}

// 도킹한 렉에서 앞으로 빠져나온다. 손으로 몰면 많이 틀어져서 만들었다.
async function jobExit(){
  const msg=$('actMsg');
  try{
    const r=await api('/job/exit',{method:'POST',
        headers:{'Content-Type':'application/json'},body:JSON.stringify({})});
    if(msg){ msg.textContent=r.message||(r.ok?'완료':'실패');
             msg.style.color = r.ok===false ? 'var(--bad)' : 'var(--dim)'; }
  }catch(e){ if(msg){ msg.textContent='요청 실패'; msg.style.color='var(--bad)'; } }
  poiJobPoll();
}

// ---------------------------------------------------------------- 조이스틱
// 스틱 위치를 그대로 twist 로 보낸다. 위=전진, 좌우=회전.
// 손을 떼면 중앙 복귀 + 즉시 정지. 놓친 명령으로 계속 달리는 사고를 막는다.
let joy={active:false, x:0, y:0, timer:null};

function drawJoy(){
  const cv=$('joy'); if(!cv||!cv.getContext) return;
  const g=cv.getContext('2d'), W=cv.width, H=cv.height;
  const cx=W/2, cy=H/2, R=Math.min(W,H)/2-16;
  g.clearRect(0,0,W,H);

  const live = (mode==='manual' && !estop);
  g.strokeStyle = live?'#2e3a4d':'#232733'; g.lineWidth=2;
  g.beginPath(); g.arc(cx,cy,R,0,7); g.stroke();
  g.strokeStyle='#1e2530'; g.lineWidth=1;
  g.beginPath(); g.arc(cx,cy,R*0.55,0,7); g.stroke();
  g.beginPath(); g.moveTo(cx-R,cy); g.lineTo(cx+R,cy);
  g.moveTo(cx,cy-R); g.lineTo(cx,cy+R); g.stroke();

  const kx=cx+joy.x*R, ky=cy-joy.y*R;
  if(joy.active){
    g.strokeStyle='rgba(77,163,255,.5)'; g.lineWidth=3;
    g.beginPath(); g.moveTo(cx,cy); g.lineTo(kx,ky); g.stroke();
  }
  g.fillStyle = !live ? '#3a4152' : joy.active ? '#4da3ff' : '#48546b';
  g.beginPath(); g.arc(kx,ky,20,0,7); g.fill();
  if(!live){
    g.fillStyle='#6f7788'; g.font='12px system-ui'; g.textAlign='center';
    g.fillText(estop?'비상정지 상태':'수동 모드에서 사용', cx, cy+R+12);
    g.textAlign='left';
  }
}

// 조이스틱 축 정리.
// ⚠ base_node 는 전진하면서 돌 때 회전 반경 1.5m 를 지킨다(w <= v/1.5).
//   옆으로 밀어도 위아래가 조금 섞이면 전진이 생겨 회전이 거의 0 으로 잘렸다
//   (2026-09-28: "좌회전 우회전이 거의 안 먹힘"). 주로 옆이면 제자리 회전,
//   주로 위아래면 곧게 간다. 가운데 작은 입력은 무시한다.
function joyAxes(){
  let x=joy.x, y=joy.y;
  if(Math.hypot(x,y) < 0.12) return [0,0];
  if(Math.abs(y) < 0.5*Math.abs(x)) y=0;      // 수평에서 약 27도 안
  if(Math.abs(x) < 0.25*Math.abs(y)) x=0;     // 수직에서 약 14도 안
  return [x,y];
}
function joySend(){
  const [ax,ay]=joyAxes();
  const lin=(+$('vLin').value)*ay;
  const ang=(+$('vAng').value)*(-ax);
  twist(lin, ang);
  const h=$('joyHint');
  if(h) h.textContent = (joy.active||lin||ang)
    ? lin.toFixed(2)+' m/s   '+ang.toFixed(2)+' rad/s' : '';
}
function joyStart(){
  if(joy.timer) return;
  joy.timer=setInterval(joySend,150);   // base_node cmd_vel 타임아웃 0.5초
}
// force 가 아니면 조이스틱으로 실제로 몰고 있을 때만 서버에 정지를 보낸다.
// ⚠ 서버의 정지는 도킹·렉 나오기 작업까지 끊는다(2026-09-28). 창이 포커스를
//   잃거나(blur) 방향키를 뗄 때마다 정지를 보내던 탓에, 손대지 않았는데
//   도킹이 '작업 취소' 로 끝났다.
function stopDrive(force){
  const driving = joy.active || !!joy.timer;
  joy.active=false; joy.x=0; joy.y=0;
  if(joy.timer){ clearInterval(joy.timer); joy.timer=null; }
  const h=$('joyHint'); if(h) h.textContent='';
  const cv=$('joy'); if(cv) cv.classList.remove('drag');
  // 손을 떼면 감속해서 선다(base_node 감속 제한을 거친다). [정지]·Space 만
  // 즉시 정지. 2026-09-28: 손을 뗄 때마다 비상정지처럼 덜컥 섰다.
  if(force===true) stopNow();
  else if(driving) twist(0,0);
  drawJoy();
}
function joySet(cx,cy,ev){
  const cv=$('joy'), r=cv.getBoundingClientRect();
  let dx=(ev.clientX-r.left)/r.width*2-1;
  let dy=1-(ev.clientY-r.top)/r.height*2;
  const m=Math.hypot(dx,dy);
  if(m>1){ dx/=m; dy/=m; }
  joy.x=dx; joy.y=dy;
}
// 조이스틱은 화면마다 복제하지 않는다. 패널이 모두 DOM 에 남아 있는 구조라
// 같은 id 가 여러 개 생기면 동작이 깨진다. 하나뿐인 요소를 활성 화면의
// 슬롯으로 옮겨서 대시보드/매핑 어느 쪽에서도 쓸 수 있게 한다.
function moveJoy(id){
  const jb=$('joyBox'); if(!jb) return;
  const slot = id==='mapping' ? $('joySlotMapping')
             : id==='dash'    ? $('joySlotDash') : null;
  if(slot && jb.parentNode!==slot){ slot.appendChild(jb); }
}

function bindJoy(){
  const cv=$('joy'); if(!cv||cv._bound) return; cv._bound=true;
  cv.addEventListener('pointerdown',e=>{
    if(mode!=='manual'||estop) return;
    e.preventDefault(); cv.setPointerCapture(e.pointerId);
    cv.classList.add('drag'); joy.active=true;
    joySet(0,0,e); joyStart(); joySend(); drawJoy();
  });
  cv.addEventListener('pointermove',e=>{
    if(!joy.active) return; joySet(0,0,e); drawJoy(); });
  cv.addEventListener('pointerup',stopDrive);
  cv.addEventListener('pointercancel',stopDrive);
  cv.addEventListener('pointerleave',()=>{ if(joy.active) stopDrive(); });
}

// 키보드는 보조 수단으로 유지
const KEYVEC={KeyW:[0,1],ArrowUp:[0,1],KeyS:[0,-1],ArrowDown:[0,-1],
              KeyA:[-1,0],ArrowLeft:[-1,0],KeyD:[1,0],ArrowRight:[1,0]};
addEventListener('keydown',e=>{
  if(e.repeat||e.target.tagName==='INPUT') return;
  if(e.code==='Space'){ e.preventDefault(); stopDrive(true); return; }
  const v=KEYVEC[e.code];
  if(v && mode==='manual' && !estop){
    e.preventDefault(); joy.active=true; joy.x=v[0]; joy.y=v[1];
    joyStart(); joySend(); drawJoy();
  }});
addEventListener('keyup',e=>{ if(KEYVEC[e.code]) stopDrive(); });
addEventListener('blur',stopDrive);

// ---------------------------------------------------------------- 맵 뷰
function mapCanvas(){ return cur==='mapping' ? $('cvMap2') : $('cvMap'); }
function mapTg(base){                     // 화면별로 다른 토글을 집는다
  return cur==='mapping' ? $(base+'2') : $(base);
}
function w2s(cv,x,y){   // 월드(m) → 화면(px). 월드 +y 가 화면 위쪽
  return [cv.width/2 + (x-view.cx)*view.ppm,
          cv.height/2 - (y-view.cy)*view.ppm];
}
function s2w(cv,sx,sy){ // 화면(px) → 월드(m). w2s 의 역변환
  const k=cv.width/cv.getBoundingClientRect().width;
  return [view.cx + (sx*k - cv.width/2)/view.ppm,
          view.cy - (sy*k - cv.height/2)/view.ppm];
}
// 위치 재조정 상태. setting 이면 지도 드래그가 '자세 지정' 으로 바뀐다.
let reloc={setting:false, x:0, y:0, yaw:0, dragging:false};
// 이동 목표. reloc 과 같은 방식이다 - 지도를 끌어 자리와 방향을 정하고
// '이동' 을 눌러야 실제로 간다. 누르기 전에는 미리보기일 뿐이다.
// setting  지도 몸짓을 받는 중 (목표를 찍는 동안만)
// showing  막대와 상태 표시가 떠 있는 중 (이동이 끝날 때까지)
// has      찍은 목표가 있어 지도에 그릴 것
let goal={setting:false, showing:false, has:false,
          x:0, y:0, yaw:0, dragging:false, timer:null};
function centerOn(x,y){
  view.cx=x; view.cy=y;
  const tf=mapTg('tgFollow'); if(tf) tf.checked=false;   // 사람이 보는 자리 고정
  drawMap();
}
function zoomBy(f){ view.ppm=Math.max(4,Math.min(400,view.ppm*f)); drawMap(); saveUI(); }
function resetView(){
  if(mapMeta&&mapMeta.available){
    const cv=mapCanvas(); if(!cv) return;
    const W=mapMeta.width*mapMeta.resolution, H=mapMeta.height*mapMeta.resolution;
    view.cx=mapMeta.origin.x+W/2; view.cy=mapMeta.origin.y+H/2;
    view.ppm=Math.min(cv.width/W, cv.height/H)*0.92;
  }else if(mapPose){ view.cx=mapPose.x; view.cy=mapPose.y; view.ppm=40; }
  else { view.cx=0; view.cy=0; view.ppm=40; }
  drawMap();
}

// 라이다로 인식한 충전 스테이션 (2026-11-13). (x, y) = 앞면 가운데, yaw = 바깥(로봇 쪽).
// 몸통 폭 CHG_W · 깊이 CHG_D (실측 폭 약 0.40m, 깊이는 사진으로 어림), 앞면 V 홈
// (입구 273mm, 깊이 83mm), 가운데 번개.
const CHG_W=0.40, CHG_D=0.30;
// 충전기 자리 (지도). 서버는 그 스캔을 찍던 자세(scan.pose) 기준으로 준다. 위치 재조정
// 미리보기처럼 다른 자세(sp)에 스캔을 얹을 때는 로봇 기준 상대 자리를 그 자세로 옮긴다.
function scanCharger(sp){
  const ch = scan && scan.charger, sp0 = scan && scan.pose;
  if(!ch || !sp0 || !sp) return ch||null;
  const dx=ch.x-sp0.x, dy=ch.y-sp0.y, c0=Math.cos(-sp0.yaw), s0=Math.sin(-sp0.yaw);
  const rx=dx*c0-dy*s0, ry=dx*s0+dy*c0, ryaw=ch.yaw-sp0.yaw;
  const c1=Math.cos(sp.yaw), s1=Math.sin(sp.yaw);
  return {x:sp.x+rx*c1-ry*s1, y:sp.y+rx*s1+ry*c1, yaw:sp.yaw+ryaw};
}
function drawCharger(g, cv, ch){
  const c=Math.cos(ch.yaw), s=Math.sin(ch.yaw);
  const P=(u,v)=>w2s(cv, ch.x+u*c-v*s, ch.y+u*s+v*c);
  const body=[[0,CHG_W/2],[0,0.137],[-0.083,0],[0,-0.137],[0,-CHG_W/2],
              [-CHG_D,-CHG_W/2],[-CHG_D,CHG_W/2]].map(([u,v])=>P(u,v));
  g.save(); g.lineJoin='round';
  g.beginPath(); body.forEach(([x,y],i)=> i?g.lineTo(x,y):g.moveTo(x,y)); g.closePath();
  g.fillStyle='rgba(235,238,242,.92)'; g.fill();
  g.lineWidth=3; g.strokeStyle='rgba(0,0,0,.6)'; g.stroke();
  g.lineWidth=1.6; g.strokeStyle='#ffb547'; g.stroke();
  const [cx,cy]=P(-CHG_D*0.55,0);
  g.fillStyle='#ffb547'; g.font='bold '+Math.max(11, Math.round(0.16*view.ppm))+'px system-ui';
  g.textAlign='center'; g.textBaseline='middle'; g.fillText('⚡', cx, cy);
  g.restore();
}

function drawMap(){
  const cv=mapCanvas(); if(!cv||!cv.getContext) return;
  const g=cv.getContext('2d'), W=cv.width, H=cv.height;
  g.fillStyle='#080a0e'; g.fillRect(0,0,W,H);

  // 맵 이미지
  if(mapImg && mapMeta && mapMeta.available){
    const m=mapMeta, wm=m.width*m.resolution, hm=m.height*m.resolution;
    const [sx,sy]=w2s(cv, m.origin.x, m.origin.y+hm);   // 좌상단
    g.imageSmoothingEnabled=false;
    g.globalAlpha=0.95;
    g.drawImage(mapImg, sx, sy, wm*view.ppm, hm*view.ppm);
    g.globalAlpha=1;
  }

  // 격자 (축척 막대가 이 간격을 쓰므로 토글과 무관하게 계산해 둔다)
  const step = view.ppm<12?5 : view.ppm<30?2 : 1;
  const gt=mapTg('tgGrid');
  if(!gt || gt.checked){
    g.strokeStyle='#1a1f2b'; g.lineWidth=1;
    const x0=view.cx-W/2/view.ppm, x1=view.cx+W/2/view.ppm;
    const y0=view.cy-H/2/view.ppm, y1=view.cy+H/2/view.ppm;
    for(let x=Math.ceil(x0/step)*step; x<=x1; x+=step){
      const [sx]=w2s(cv,x,0); g.beginPath(); g.moveTo(sx,0); g.lineTo(sx,H); g.stroke(); }
    for(let y=Math.ceil(y0/step)*step; y<=y1; y+=step){
      const [,sy]=w2s(cv,0,y); g.beginPath(); g.moveTo(0,sy); g.lineTo(W,sy); g.stroke(); }
  }

  const p = mapPose;

  // 이동 궤적
  if(p && mapTg('tgTrail') && mapTg('tgTrail').checked && trail.length>1){
    g.strokeStyle='rgba(77,163,255,.55)'; g.lineWidth=2; g.beginPath();
    trail.forEach(([x,y],i)=>{ const [sx,sy]=w2s(cv,x,y);
      i?g.lineTo(sx,sy):g.moveTo(sx,sy); });
    g.stroke();
  }

  // 라이다 (로봇 자세로 회전시켜 맵 좌표에 올린다)
  if(p && scan && scan.points && mapTg('tgScan') && mapTg('tgScan').checked){
    // 스캔 각도는 라이다 프레임 기준이다. 라이다가 로봇에 대해 틀어져
    // 달려 있으면(실측 2.1도) 그만큼 더해 줘야 지도와 맞는다.
    const mt = scan.mount || {x:0,y:0,yaw:0};
    // 스캔을 '지금' 자세에 얹으면 회전 중에 점이 통째로 돌아 보인다.
    // 스캔과 자세를 따로 받아오므로 100ms 쯤 시점이 어긋나는데, 0.4 rad/s
    // 에서 그만큼이면 2.3도이고 10m 앞에서 40cm 다. api_node 가 그 스캔을
    // 찍던 순간의 자세를 같이 보내 주므로 그것을 쓴다.
    // 2026-11-13 위치 재조정 중에는 라이다를 옮기고 있는 로봇(미리보기)에 붙여 그린다 —
    // 끌면 점도 같이 움직여 지도 벽에 맞는 자리를 눈으로 찾는다 (사용자: "수동위치 조정
    // 움직일때 라이다도 같이 움직여서 매칭할수있게").
    const sp = (reloc.setting || reloc.dragging) ? {x:reloc.x, y:reloc.y, yaw:reloc.yaw}
                                                 : (scan.pose || p);
    const cs=Math.cos(sp.yaw), sn=Math.sin(sp.yaw);
    const lx = sp.x + mt.x*cs - mt.y*sn;     // 라이다의 맵 좌표
    const ly = sp.y + mt.x*sn + mt.y*cs;
    // 충전기로 인식됐으면 그 자리의 V 점은 감추고 아래에서 충전기 모양을 그린다
    const ch = scanCharger(sp);
    const cc = ch ? Math.cos(ch.yaw) : 0, cn = ch ? Math.sin(ch.yaw) : 0;
    for(const [ad,d] of scan.points){
      const a = sp.yaw + mt.yaw + ad*Math.PI/180;
      const wx = lx+Math.cos(a)*d, wy = ly+Math.sin(a)*d;
      if(ch){
        const u=(wx-ch.x)*cc+(wy-ch.y)*cn, v=-(wx-ch.x)*cn+(wy-ch.y)*cc;
        if(u>-CHG_D-0.03 && u<0.04 && Math.abs(v)<CHG_W/2+0.03) continue;
      }
      const [sx,sy]=w2s(cv, wx, wy);
      if(sx<0||sy<0||sx>W||sy>H) continue;
      g.fillStyle = d<0.5?'#ff5c5c' : d<1.0?'#ffb454' : 'rgba(77,163,255,.9)';
      g.fillRect(sx-1.5,sy-1.5,3,3);
    }
  }

  if(p && scan && scan.charger){
    const spc = (reloc.setting || reloc.dragging) ? {x:reloc.x, y:reloc.y, yaw:reloc.yaw}
                                                  : (scan.pose || p);
    const chm = scanCharger(spc); if(chm) drawCharger(g, cv, chm);
  }

  // 인식된 렉(대차) — 다리 네 개와 그 사각형
  //   초록 = 카메라 검증 통과 / 주황 = 실패 / 노랑 = 판정 보류(화각 밖)
  // 렉을 못 찾고 있을 때만 이유를 적는다. 잘 되고 있을 때는 사각형이
  // 곧 답이므로 글자를 더 얹지 않는다.
  if(cartOn){
    const bad = !carts ? ('자료 없음'+(cartsErr?' — '+cartsErr:''))
              : !carts.carts ? '응답 형식 이상'
              : carts.carts.length===0 ? '인식 없음 (다리 후보 '+(carts.candidates||0)+'개)'
              : carts.stale ? '자료가 오래됨 — 검출 노드 확인'
              : null;
    if(bad){
      g.fillStyle='#8f97a8'; g.font='11px ui-monospace,monospace';
      g.fillText('렉 인식: '+bad, 12, 18);
    }
  }
  if(cartOn && carts && carts.carts){
    for(const c of carts.carts){
      // 서버가 준 map 좌표를 쓰되, TF 조회가 실패해 빠졌으면 로봇 자세로
      // 직접 옮긴다. 대차 좌표는 항상 라이다(로봇) 기준으로 들어온다.
      let legsM=null, ctrM=null;
      if(c.map && c.map.legs && c.map.legs.length>=4){
        legsM=c.map.legs; ctrM=c.map.xy;
      }else if(p){
        const cs=Math.cos(p.yaw), sn=Math.sin(p.yaw);
        const tf=(lx,ly)=>[p.x+lx*cs-ly*sn, p.y+lx*sn+ly*cs];
        legsM=c.legs.map(([lx,ly])=>tf(lx,ly));
        ctrM=tf(c.x,c.y);
      }
      if(!legsM) continue;
      const col = c.cam_ok===false ? '#ff8f5c'
                : c.cam_ok===true  ? '#4de0a0' : '#ffd23f';
      // 들어갈 대상 하나만 진하게. 나머지는 지우지 않고 흐리게 둔다 -
      // 방 안의 다른 물건 넷이 우연히 규격에 맞아 함께 잡히는 일이 있어,
      // 왜 이것을 골랐는지 눈으로 볼 수 있어야 한다.
      const tgt = c.target !== false;
      g.globalAlpha = tgt ? 1 : 0.38;
      g.beginPath();
      legsM.forEach(([lx,ly],i)=>{ const [sx,sy]=w2s(cv,lx,ly);
        i?g.lineTo(sx,sy):g.moveTo(sx,sy); });
      g.closePath();
      g.fillStyle=tgt?'rgba(255,255,255,.08)':'rgba(255,255,255,.03)'; g.fill();
      g.strokeStyle=col; g.lineWidth=tgt?3:1.5;
      if(!tgt) g.setLineDash([5,4]);
      g.stroke(); g.setLineDash([]);
      g.fillStyle=col;
      for(const [lx,ly] of legsM){
        const [sx,sy]=w2s(cv,lx,ly);
        g.beginPath(); g.arc(sx,sy,Math.max(4,0.04*view.ppm),0,7); g.fill();
        g.strokeStyle='#0f1116'; g.lineWidth=1.5; g.stroke();
      }
      const [cxp,cyp]=w2s(cv,ctrM[0],ctrM[1]);
      g.fillStyle=col;
      g.font='12px system-ui'; g.textAlign='center';
      g.fillText(c.cart+(c.cam_ok===true?' ✓':c.cam_ok===false?' ?':''),cxp,cyp-6);
      g.font='11px ui-monospace,monospace';
      g.fillText((c.score*100).toFixed(0)+'%'+(tgt?' · 대상':''),cxp,cyp+12);
      g.textAlign='left';
      g.globalAlpha=1;
    }
  }

  // 지도 수정 중이면 지울 자리를 덧그린다.
  //   연한 초록 = 이미 적용된 것 / 노랑 = 아직 확인 안 누른 것
  //
  // 원을 하나씩 채우면 겹친 만큼 색이 쌓여 금세 불투명한 덩어리가 된다.
  // 획을 조금만 그어도 지도가 안 보였다. 원들을 '한 경로' 로 모아 한 번에
  // 채우면 겹쳐도 색이 진해지지 않아 지운 자리가 평평하게 보인다.
  //
  // 이미 적용된 자리는 지도에 흰색으로 이미 반영돼 있다. 여기 초록은
  // '되돌릴 수 있는 자리' 라는 표시일 뿐이므로 아주 연하게만 둔다.
  if(edit.on){
    const paint=(list,op,fill)=>{
      const path=new Path2D(); let n=0;
      for(const sp of list){
        if(!sp || sp.kind!=='circle') continue;
        if((sp.op||'erase')!==op) continue;
        const [sx,sy]=w2s(cv,sp.x,sp.y);
        const rp=Math.max(2, sp.r*view.ppm);
        path.moveTo(sx+rp,sy); path.arc(sx,sy,rp,0,7); n++;
      }
      if(!n) return;
      g.fillStyle=fill; g.fill(path);
    };
    // 선(가상벽)과 영역(금지구역)
    const drawShape=(sp,stroke,fill)=>{
      const P=(sp.points||[]).map(([x,y])=>w2s(cv,x,y));
      if(P.length<2) return;
      g.beginPath();
      P.forEach(([sx,sy],i)=>{ i?g.lineTo(sx,sy):g.moveTo(sx,sy); });
      if(sp.kind==='poly'){
        g.closePath(); g.fillStyle=fill; g.fill();
        g.strokeStyle=stroke; g.lineWidth=2; g.stroke();
      }else{
        g.strokeStyle=stroke; g.lineCap='round'; g.lineJoin='round';
        g.lineWidth=Math.max(2,(sp.width||0.15)*view.ppm); g.stroke();
        g.lineCap='butt'; g.lineJoin='miter';
      }
      // 점을 보여 줘야 어디를 찍었는지 알 수 있다
      g.fillStyle=stroke;
      for(const [sx,sy] of P){ g.beginPath(); g.arc(sx,sy,3,0,7); g.fill(); }
    };
    // 적용된 것은 연하게(지도에 이미 반영돼 있다), 확인 전은 진하게
    //   지우개 = 초록/노랑 · 가상벽·금지구역 = 파랑
    const pend=[].concat(...edit.pend);
    paint(edit.saved,'erase','rgba(77,224,160,.16)');
    paint(pend,'erase','rgba(255,190,40,.38)');
    for(const sp of edit.saved)
      if(sp && (sp.kind==='line'||sp.kind==='poly'))
        drawShape(sp,'rgba(90,150,255,.55)','rgba(90,150,255,.14)');
    for(const sp of pend)
      if(sp && (sp.kind==='line'||sp.kind==='poly'))
        drawShape(sp,'rgba(120,180,255,.95)','rgba(120,180,255,.28)');

    // 지금 찍고 있는 점들
    if(edit.pts.length){
      const P=edit.pts.map(([x,y])=>w2s(cv,x,y));
      g.strokeStyle='#ffd23f'; g.lineWidth=2; g.setLineDash([6,4]);
      g.beginPath();
      P.forEach(([sx,sy],i)=>{ i?g.lineTo(sx,sy):g.moveTo(sx,sy); });
      if(edit.mode==='zone' && P.length>2) g.closePath();
      g.stroke(); g.setLineDash([]);
      g.fillStyle='#ffd23f';
      P.forEach(([sx,sy],i)=>{
        g.beginPath(); g.arc(sx,sy,4,0,7); g.fill();
        g.font='10px ui-monospace,monospace'; g.fillText(i+1, sx+6, sy-6);
      });
    }
  }

  // 로봇 — 실제 차체 크기로 그린다. 확대하면 로봇도 같이 커진다.
  // 예전에는 반지름을 26px 로 상한을 둬서 아무리 확대해도 커지지 않았다.
  if(p){
    const [sx,sy]=w2s(cv,p.x,p.y);
    // 실제 차체 크기 그대로. 확대하면 같이 커진다.
    // (화면이 로봇을 따라가고 확대도 되므로 억지로 키우지 않는다)
    const k=1;
    const L=ROBOT.len*view.ppm, Wd=ROBOT.wid*view.ppm;
    g.save(); g.translate(sx,sy); g.rotate(-p.yaw);
    if(L>=14){
      // 구동축(=로봇 원점)이 앞끝에서 ROBOT.front 만큼 뒤에 있다
      const fx=ROBOT.front*view.ppm*k, bx=L-fx, hy=Wd/2;
      g.fillStyle='rgba(61,220,151,.20)';
      g.strokeStyle='#3ddc97'; g.lineWidth=2;
      g.beginPath(); g.rect(-bx,-hy,L,Wd); g.fill(); g.stroke();
      // 진행 방향 화살표
      g.fillStyle='#3ddc97';
      const a=Math.min(hy*0.9, L*0.28);
      g.beginPath(); g.moveTo(fx,0);
      g.lineTo(fx-a,-hy*0.7); g.lineTo(fx-a,hy*0.7); g.closePath(); g.fill();
      // 구동축 중심 표시 (좌표 기준점)
      g.fillStyle='#0f1116';
      g.beginPath(); g.arc(0,0,Math.max(2,Math.min(4,L*0.04)),0,7); g.fill();
    }else{
      // 너무 축소돼 차체가 점만 해지면 최소 표식으로 바꿔 보이게 유지
      const r=7;
      g.fillStyle='rgba(61,220,151,.25)';
      g.beginPath(); g.arc(0,0,r,0,7); g.fill();
      g.strokeStyle='#3ddc97'; g.lineWidth=2; g.stroke();
      g.fillStyle='#3ddc97'; g.beginPath();
      g.moveTo(r,0); g.lineTo(-r*0.5,-r*0.62); g.lineTo(-r*0.5,r*0.62);
      g.closePath(); g.fill();
    }
    g.restore();
  }else{
    g.fillStyle='#5c6577'; g.font='13px system-ui'; g.textAlign='center';
    g.fillText('로봇 위치 없음 — SLAM(map 프레임)이 필요합니다', W/2, H/2);
    g.textAlign='left';
  }

  // POI.
  //
  // 대차는 등록된 규격대로 실제 크기의 네모를 그린다. 화면 고정 크기로
  // 그리면 확대했을 때 렉이 어디까지 차지하는지 가늠이 안 된다. 방향도
  // 같이 보여야 어느 쪽으로 들어가는지 알 수 있다.
  //
  // 규격의 어느 변이 진입축인지는 다리 넷만으로는 알 수 없다(90도 주기).
  // 깊이(앞뒤 다리 간격)를 진입축에 놓고 그린다 — 650 대 574 라 어느 쪽이든
  // 크게 어긋나 보이지 않는다.
  if(poiList.length){
    g.save();
    g.lineJoin='round'; g.lineCap='round';
    // ⚠ 2026-11-13 캔버스(960px)를 화면에서 줄여 보여 주므로 고정 px 은 그 비율(ck)을
    //   곱해야 화면에서 그 크기다. 안 곱하면 마커 · 글자가 2/3 로 작게 보였다
    //   (사용자: "지도 안의 지점 poi 마커랑 텍스트가 다 작아").
    const ck=cv.width/Math.max(1, cv.getBoundingClientRect().width||cv.width);
    for(const q of poiList){
      const [px,py]=w2s(cv,q.x,q.y);
      const cart = q.kind==='cart';
      // 색은 보라다. 인식 결과가 주황(#ff8f5c)·초록(#4de0a0)·노랑(#ffd23f)을
      // 쓰고 있어 그 계열을 쓰면 '등록해 둔 자리' 와 '지금 보이는 대차' 가
      // 구분되지 않는다. 경로(자홍)·궤적(파랑)·로봇(초록)과도 안 겹친다.
      const col = cart ? '#b892ff' : q.kind==='charger' ? '#ffb547' : '#57d0ff';
      if(cart){
        // 등록할 때 라이다로 잰 값이 있으면 그것을 쓴다. 규격표는 어느 변이
        // 진입축인지 모르지만, 실측값은 로봇이 바라보던 방향 기준으로
        // [앞뒤, 좌우] 순서로 저장돼 있다.
        const sp = cartSpec[q.cart] || {w:0.65, d:0.60};
        const hd = (q.size ? q.size[0] : sp.d)/2;   // 진입축 반길이
        const hw = (q.size ? q.size[1] : sp.w)/2;   // 좌우 반폭
        // 등록할 때 인식된 다리 네 점이 있으면 그것을 그대로 그린다.
        // 크기 두 개로는 비직각 렉의 모양을 못 담는다.
        const cs=Math.cos(q.yaw), sn=Math.sin(q.yaw);
        // 다리 네 점이 있으면 그것을 감싸는 네모를 그린다.
        //
        // ⚠ 네 점을 그대로 이으면 다리 순서에 따라 X 자로 꼬여 보인다.
        //   그리고 POI 는 '렉이 차지하는 자리' 를 보여 주는 것이지 다리
        //   모양을 보여 주는 것이 아니다. 네모만 그린다(2026-09-21).
        // ⚠ 이름을 bx0/bx1 로 둔다. a1 은 아래 화살촉이 이미 쓰고 있어,
        //   같은 블록에서 두 번 선언하면 스크립트 전체가 안 돌아간다
        //   (2026-09-21: 관리 페이지가 사이드바만 남고 비었다).
        let bx0=-hd, bx1=hd, by0=-hw, by1=hw;
        if(q.legs && q.legs.length===4){
          const A=q.legs.map(p=>p[0]), B=q.legs.map(p=>p[1]);
          bx0=Math.min(...A); bx1=Math.max(...A);
          by0=Math.min(...B); by1=Math.max(...B);
        }
        const pts=[[bx1,by1],[bx1,by0],[bx0,by0],[bx0,by1]].map(([a,b])=>
          w2s(cv, q.x + a*cs - b*sn, q.y + a*sn + b*cs));
        g.beginPath();
        pts.forEach(([sx,sy],i)=> i?g.lineTo(sx,sy):g.moveTo(sx,sy));
        g.closePath();
        g.fillStyle='rgba(184,146,255,.12)'; g.fill();
        g.lineWidth=3*ck; g.strokeStyle='rgba(0,0,0,.55)'; g.stroke();
        g.setLineDash([6*ck,4*ck]);
        g.lineWidth=2*ck; g.strokeStyle=col; g.stroke();
        g.setLineDash([]);
        // 진입 방향 (로봇이 나오는 쪽). 화살표로 그려 어느 쪽인지 분명하게.
        const L=Math.max(22*ck, hd*view.ppm*1.1);
        const ca=Math.cos(q.yaw), sa=-Math.sin(q.yaw);   // 화면 y 는 아래로
        const ex=px+L*ca, ey=py+L*sa;
        const hlen=Math.max(9*ck, L*0.30), hwid=Math.max(5*ck, L*0.16);
        // 촉의 밑변 두 점
        const bx=ex-hlen*ca, by=ey-hlen*sa;
        const a1=[bx-hwid*sa, by+hwid*ca], a2=[bx+hwid*sa, by-hwid*ca];
        for(const pass of [0,1]){
          g.lineWidth = (pass? 2 : 3.6)*ck;
          g.strokeStyle = pass? col : 'rgba(0,0,0,.55)';
          g.beginPath(); g.moveTo(px,py); g.lineTo(bx,by); g.stroke();
          g.beginPath(); g.moveTo(ex,ey); g.lineTo(a1[0],a1[1]);
          g.lineTo(a2[0],a2[1]); g.closePath();
          if(pass){ g.fillStyle=col; g.fill(); } else g.stroke();
        }
      }else if(q.kind==='charger'){
        // 충전소 POI = 도킹을 마친 로봇 자세. 로봇 크기(0.8735 x 0.3288m) 점선 박스와
        // 로봇이 바라보는 쪽(스테이션 반대) 화살표 (2026-11-13, 사용자 제안).
        // 실제 스테이션은 라이다로 보이면 drawCharger 가 따로 그린다.
        const cs=Math.cos(q.yaw), sn=Math.sin(q.yaw), CC='#ffb547';
        const hl=0.43675, hw=0.1644;
        const box=[[hl,hw],[hl,-hw],[-hl,-hw],[-hl,hw]].map(([a,b])=>
          w2s(cv, q.x + a*cs - b*sn, q.y + a*sn + b*cs));
        g.beginPath(); box.forEach(([sx,sy],i)=> i?g.lineTo(sx,sy):g.moveTo(sx,sy)); g.closePath();
        g.fillStyle='rgba(255,181,71,.12)'; g.fill();
        g.lineWidth=3*ck; g.strokeStyle='rgba(0,0,0,.55)'; g.stroke();
        g.setLineDash([6*ck,4*ck]); g.lineWidth=2*ck; g.strokeStyle=CC; g.stroke(); g.setLineDash([]);
        // 앞쪽 화살촉 (차체 앞끝 안쪽)
        const [tx,ty]=w2s(cv, q.x + hl*0.95*cs, q.y + hl*0.95*sn);
        const [b1x,b1y]=w2s(cv, q.x + (hl*0.6)*cs - hw*0.7*sn, q.y + (hl*0.6)*sn + hw*0.7*cs);
        const [b2x,b2y]=w2s(cv, q.x + (hl*0.6)*cs + hw*0.7*sn, q.y + (hl*0.6)*sn - hw*0.7*cs);
        g.beginPath(); g.moveTo(tx,ty); g.lineTo(b1x,b1y); g.lineTo(b2x,b2y); g.closePath();
        g.fillStyle=CC; g.fill();
      }else{
        // 지점: 화면에서 반지름 8px 동그라미 + 가운데 점
        g.beginPath(); g.arc(px,py,8*ck,0,Math.PI*2);
        g.fillStyle='rgba(87,208,255,.18)'; g.fill();
        g.lineWidth=3.5*ck; g.strokeStyle='rgba(0,0,0,.6)'; g.stroke();
        g.lineWidth=2.2*ck; g.strokeStyle=col; g.stroke();
        g.beginPath(); g.arc(px,py,2.2*ck,0,Math.PI*2); g.fillStyle=col; g.fill();
      }
      // 2026-11-13 11px -> 14px 굵게, 확대하면 18px 까지 (사용자: "텍스트 사이즈도 너무 작음")
      const fs=Math.round(Math.max(14, Math.min(18, 0.06*view.ppm))*ck);
      g.font='bold '+fs+'px system-ui'; g.textAlign='center'; g.textBaseline='alphabetic';
      g.lineWidth=4*ck; g.strokeStyle='rgba(0,0,0,.8)';
      const ty = (q.kind==='point') ? py-12*ck : py-6*ck;   // 지점은 동그라미 위로
      g.strokeText(q.name, px, ty);
      g.fillStyle=col; g.fillText(q.name, px, ty);
    }
    g.textAlign='left';
    g.restore();
  }

  // nav2 가 낸 경로. 지나온 자취보다 위에, 로봇보다 아래에 그린다.
  //
  // 색은 자홍이다. 지도에 이미 파랑(자취)·초록(로봇)·노랑(재조정)이 있어
  // 그 계열을 쓰면 묻힌다. 그리고 지도 자체가 흰색(자유)과 검정(벽)을
  // 오가므로, 선 하나만 그으면 어느 한쪽에서 반드시 안 보인다.
  // 굵은 검은 선을 먼저 깔고 그 위에 자홍을 얹어 양쪽 다 뜨게 한다.
  if(navPath.length>1){
    const PC='#ff3ea5';
    g.save();
    g.lineJoin='round'; g.lineCap='round';
    const trace=()=>{
      g.beginPath();
      for(let i=0;i<navPath.length;i++){
        const [px,py]=w2s(cv,navPath[i][0],navPath[i][1]);
        i ? g.lineTo(px,py) : g.moveTo(px,py);
      }
    };
    trace(); g.strokeStyle='rgba(0,0,0,.55)'; g.lineWidth=4; g.stroke();
    trace(); g.strokeStyle=PC;               g.lineWidth=2; g.stroke();

    // 진행 방향을 알 수 있게 중간중간 화살촉을 찍는다
    for(let i=1;i<navPath.length;i++){
      const [ax,ay]=w2s(cv,navPath[i-1][0],navPath[i-1][1]);
      const [bx,by]=w2s(cv,navPath[i][0],navPath[i][1]);
      if(Math.hypot(bx-ax,by-ay)<44) continue;
      const th=Math.atan2(by-ay,bx-ax);
      g.save(); g.translate((ax+bx)/2,(ay+by)/2); g.rotate(th);
      g.beginPath(); g.moveTo(6,0); g.lineTo(-4,-4); g.lineTo(-4,4);
      g.closePath();
      g.fillStyle=PC; g.fill();
      g.strokeStyle='rgba(0,0,0,.55)'; g.lineWidth=1; g.stroke();
      g.restore();
    }
    // 끝점 — 흰 테를 둘러 어디서든 뜨게 한다
    const [ex,ey]=w2s(cv,navPath[navPath.length-1][0],navPath[navPath.length-1][1]);
    g.beginPath(); g.arc(ex,ey,4.5,0,7);
    g.fillStyle=PC; g.fill();
    g.strokeStyle='#fff'; g.lineWidth=1.5; g.stroke();
    g.restore();
  }

  // 이동 목표 미리보기. 깃발 하나와 바라볼 방향 화살표로 둔다.
  // 재조정처럼 배경을 어둡게 깔지는 않는다 - 목표를 정하는 동안에도
  // 로봇과 주변 지형이 같이 보여야 어디로 보낼지 판단이 된다.
  if(goal.has){
    const [gx,gy]=w2s(cv,goal.x,goal.y);
    const C='#4cc9f0';
    g.save();
    g.translate(gx,gy);
    // 목표 반경 = goal checker 의 xy_goal_tolerance(0.15m)
    g.strokeStyle=C; g.lineWidth=1.5; g.setLineDash([4,4]);
    g.beginPath(); g.arc(0,0,Math.max(6,0.15*view.ppm),0,7); g.stroke();
    g.setLineDash([]);
    // 바라볼 방향
    g.rotate(-goal.yaw);
    const L=Math.max(26, 0.6*view.ppm);
    g.strokeStyle=C; g.lineWidth=3;
    g.beginPath(); g.moveTo(0,0); g.lineTo(L,0); g.stroke();
    g.fillStyle=C;
    g.beginPath(); g.moveTo(L+9,0); g.lineTo(L-4,-6); g.lineTo(L-4,6);
    g.closePath(); g.fill();
    g.restore();
    // 십자 표적
    g.strokeStyle=C; g.lineWidth=1;
    g.beginPath();
    g.moveTo(gx-14,gy); g.lineTo(gx+14,gy);
    g.moveTo(gx,gy-14); g.lineTo(gx,gy+14);
    g.stroke();
    g.fillStyle=C; g.textAlign='center';
    g.font='bold '+Math.round(14*cv.width/Math.max(1, cv.getBoundingClientRect().width||cv.width))+'px system-ui';
    g.fillText(goal.x.toFixed(2)+', '+goal.y.toFixed(2)+
               '  '+(goal.yaw*180/Math.PI).toFixed(0)+'°', gx, gy-20);
    g.textAlign='left';
  }

  // 위치 재조정 미리보기.
  // 지도 위에 얇은 윤곽만 그리면 잘 안 보여서, 배경을 한 겹 어둡게 깔고
  // 굵은 윤곽 + 방향선 + 십자 표적으로 눈에 띄게 한다.
  if(reloc.setting || reloc.dragging){
    g.fillStyle='rgba(6,8,12,.45)'; g.fillRect(0,0,W,H);

    const [sx,sy]=w2s(cv,reloc.x,reloc.y);
    // 미리보기도 실제 크기로. 눈에 띄게 하는 몫은 배경 어둡게 + 십자선 +
    // 방향선이 맡는다. 다만 완전히 사라지지는 않게 최소치만 둔다.
    const MINPX=18;
    const k=Math.max(1, MINPX/(ROBOT.len*view.ppm));
    const L=ROBOT.len*view.ppm*k, Wd=ROBOT.wid*view.ppm*k;
    const fx=ROBOT.front*view.ppm*k, bx=L-fx, hy=Wd/2;
    const C='#ffd23f';

    // 십자 표적 — 위치가 어디인지 한눈에
    g.strokeStyle='rgba(255,210,63,.55)'; g.lineWidth=1;
    g.setLineDash([5,5]);
    g.beginPath(); g.moveTo(sx,0); g.lineTo(sx,H);
    g.moveTo(0,sy); g.lineTo(W,sy); g.stroke();
    g.setLineDash([]);

    g.save(); g.translate(sx,sy);

    // 바라볼 방향으로 뻗는 선
    g.rotate(-reloc.yaw);
    g.strokeStyle=C; g.lineWidth=2; g.setLineDash([8,5]);
    g.beginPath(); g.moveTo(fx,0); g.lineTo(fx+Math.max(60,L*1.2),0); g.stroke();
    g.setLineDash([]);

    // 차체 — 채우고 굵게 두른다
    g.fillStyle='rgba(255,210,63,.28)';
    g.beginPath(); g.rect(-bx,-hy,L,Wd); g.fill();
    g.strokeStyle=C; g.lineWidth=3; g.stroke();

    // 진행 방향 삼각형
    g.fillStyle=C;
    const a=Math.min(hy*1.1, L*0.3);
    g.beginPath(); g.moveTo(fx,0);
    g.lineTo(fx-a,-hy*0.85); g.lineTo(fx-a,hy*0.85); g.closePath(); g.fill();
    g.restore();

    // 중심점
    g.fillStyle=C; g.beginPath(); g.arc(sx,sy,4,0,7); g.fill();
    g.strokeStyle='#0f1116'; g.lineWidth=1.5; g.stroke();

    // 좌표 안내
    g.fillStyle=C; g.font='12px ui-monospace,monospace';
    g.fillText('x %s  y %s  %s°'
        .replace('%s',reloc.x.toFixed(2)).replace('%s',reloc.y.toFixed(2))
        .replace('%s',(reloc.yaw*180/Math.PI).toFixed(0)),
      sx+12, sy-12);
  }

  // 축척 막대
  const barM = step, barPx = barM*view.ppm;
  g.strokeStyle='#8f97a8'; g.lineWidth=2;
  g.beginPath(); g.moveTo(16,H-18); g.lineTo(16+barPx,H-18);
  g.moveTo(16,H-23); g.lineTo(16,H-13);
  g.moveTo(16+barPx,H-23); g.lineTo(16+barPx,H-13); g.stroke();
  g.fillStyle='#8f97a8'; g.font='11px ui-monospace,monospace';
  g.fillText(barM+' m', 20+barPx, H-14);
}

// 지도에서 로봇을 눌렀는지 (차체 크기 안쪽인지) 본다
function hitRobot(cv,sx,sy){
  if(!mapPose) return false;
  const [rx,ry]=w2s(cv,mapPose.x,mapPose.y);
  const r=Math.max(14, ROBOT.len*view.ppm*0.6);
  return Math.hypot(sx-rx, sy-ry) <= r;
}
// 위치 재조정 툴바.
//   위치 - 지도에서 끌어 로봇을 옮긴다
//   회전 - 끌어서 바라볼 방향을 돌린다
//   자동 - 스캔을 지도와 맞춰 후보를 찾아 미리보기에 넣는다
//   확인 - 그때까지 잡아 둔 자세를 실제로 반영한다
// 확인을 누르기 전에는 아무것도 반영되지 않는다(전부 미리보기).
function relocBar(){ return cur==='mapping' ? $('relocBar2') : $('relocBar'); }
function rbEl(base){ return cur==='mapping' ? $(base+'2') : $(base); }

function relocOpen(){
  reloc.setting=true; reloc.mode='move'; reloc.src='manual';
  if(mapPose){ reloc.x=mapPose.x; reloc.y=mapPose.y; reloc.yaw=mapPose.yaw; }
  const b=relocBar(); if(b) b.classList.add('on');
  relocMode('move');
  rbSay('로봇을 놓을 자리를 지도에서 끌어 주세요');
  drawMap();
}
function relocMode(m){
  reloc.mode=m;
  const mv=rbEl('rbMove'), rt=rbEl('rbRot');
  if(mv) mv.className='rb'+(m==='move'?' sel':'');
  if(rt) rt.className='rb'+(m==='rot'?' sel':'');
  rbSay(m==='move' ? '지도를 끈 만큼 로봇이 옮겨집니다 (미세: ▲▼◀▶)'
                   : '좌우로 끈 만큼 돕니다 (미세: ⟲⟳)');
  drawMap();
}
function rbSay(msg, kind){
  const e=rbEl('rbMsg'); if(!e) return;
  e.textContent=msg||'';
  e.style.color = kind==='ok' ? 'var(--ok)' : kind==='bad' ? 'var(--bad)' : 'var(--dim)';
}
function relocCancel(){
  reloc.setting=false; reloc.dragging=false; reloc.mode=null;
  const b=relocBar(); if(b) b.classList.remove('on');
  drawMap(); refreshMap();
}
async function relocRelease(){
  rbSay('라이다 보정 켜는 중...');
  try{ const r=await api('/localization/release',{method:'POST',
      headers:{'Content-Type':'application/json'},body:'{}'});
    rbSay(r.message, r.ok?'ok':'bad'); holdPoll(); }catch(e){ rbSay('요청 실패','bad'); }
}
// 2026-11-13 상단 '수동 위치 고정' 표시는 뺐다 (사용자: "확인 눌렀는데 수동위치고정 텍스트가
// 계속 상단에 떠있음"). 손으로 놓은 자리는 그대로 유지되고, [자동] · [보정 켜기] 로 라이다
// 보정을 다시 켠다.
function holdPoll(){}
async function relocAuto(){
  rbSay('지도를 훑는 중...');
  try{
    const r=await api('/localization/auto',{method:'POST',
      headers:{'Content-Type':'application/json'},body:'{}'});
    if(r.ok && r.pose){
      // 바로 반영하지 않고 미리보기에만 올린다. 확인을 눌러야 적용된다.
      reloc.x=r.pose.x; reloc.y=r.pose.y; reloc.yaw=r.pose.yaw; reloc.src='auto';
      // 찾은 자리가 화면 밖이면(수십 m 떨어진 경우가 흔하다) 미리보기가
      // 보이지 않는다. 그 자리로 화면을 옮겨 준다.
      centerOn(reloc.x, reloc.y);
      rbSay(r.message,'ok'); drawMap();
    }else{ rbSay(r.message,'bad'); }
  }catch(e){ rbSay('요청 실패','bad'); }
}
async function relocConfirm(){
  rbSay('적용 중...');
  try{
    const r=await api('/localization/manual',{method:'POST',
      headers:{'Content-Type':'application/json'},
      // 손으로 옮긴 자리는 고정(라이다 보정 끔), [자동] 후보 그대로면 라이다로 다시 맞춤
      body:JSON.stringify({x:reloc.x,y:reloc.y,yaw:reloc.yaw,hold:reloc.src!=='auto'})});
    rbSay(r.message, r.ok?'ok':'bad');
    if(r.ok){
      clearTrail();
      // 적용됐으면 화면도 새 위치를 따라가게 되돌린다
      const tf=mapTg('tgFollow'); if(tf) tf.checked=true;
      setTimeout(relocCancel, 900);
    }
  }catch(e){ rbSay('요청 실패','bad'); }
}

// ---- 이동 목표 ------------------------------------------------------
function gbSay(msg, kind){
  const e=$('gbMsg'); if(!e) return;
  e.textContent=msg||'';
  e.style.color = kind==='ok' ? 'var(--ok)' : kind==='bad' ? 'var(--bad)' : 'var(--dim)';
}
// RViz 의 2D Nav Goal 과 같은 몸짓 하나로 끝낸다.
//   누른 자리 = 목적지,  끄는 방향 = 도착했을 때 바라볼 방향,  놓으면 출발.
// 모드 버튼을 오가며 여러 번 누르는 것보다 훨씬 빠르다.
function goalOpen(){
  if(reloc.setting) relocCancel();
  goal.setting=true; goal.showing=true; goal.has=false;
  const b=$('goalBar'); if(b) b.classList.add('on');
  gbSay('지도에서 갈 자리를 누른 채 끌어 방향을 정하고 놓으세요');
  goalPoll();
  drawMap();
}
function goalCancel(){
  goal.setting=false; goal.showing=false; goal.has=false; goal.dragging=false;
  if(goal.timer){ clearInterval(goal.timer); goal.timer=null; }
  const b=$('goalBar'); if(b) b.classList.remove('on');
  drawMap();
}
async function goalSend(){
  gbSay('보내는 중...');
  try{
    const r=await api('/nav/goto',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({x:goal.x,y:goal.y,yaw:goal.yaw})});
    gbSay(r.message, r.ok?'ok':'bad');
  }catch(e){ gbSay('요청 실패','bad'); }
}
async function goalStop(){
  try{
    const r=await api('/nav/cancel',{method:'POST',
      headers:{'Content-Type':'application/json'},body:'{}'});
    gbSay(r.message,'ok');
  }catch(e){ gbSay('요청 실패','bad'); }
}
// 이동 중에는 상태를 계속 보여 준다. 'active' 면 정지 버튼이 뜬다.
function goalPoll(){
  if(goal.timer) clearInterval(goal.timer);
  goal.timer=setInterval(async()=>{
    if(!goal.showing) return;
    try{
      const d=await api('/nav/status');
      navSpeedSync(d.speed);
      const go=$('goalBar'); if(!go) return;
      let st=$('gbStop');
      if(d.state==='active'){
        if(!st){
          st=document.createElement('button');
          st.id='gbStop'; st.className='rb'; st.textContent='정지';
          st.style.color='var(--bad)';
          st.onclick=goalStop;
          go.insertBefore(st, $('gbMsg'));
        }
        gbSay('이동 중...');
      }else{
        if(st) st.remove();
        if(d.state==='done')     gbSay(d.message,'ok');
        else if(d.state==='failed' || d.state==='canceled') gbSay(d.message,'bad');
      }
    }catch(e){}
  }, 700);
}
function goalDown(cv,e){
  const r=cv.getBoundingClientRect();
  const [wx,wy]=s2w(cv, e.clientX-r.left, e.clientY-r.top);
  goal.x=wx; goal.y=wy; goal.has=true;
  // 끌지 않고 놓으면 '로봇이 그쪽을 보고 도착' 이 가장 자연스럽다.
  goal.yaw = mapPose ? Math.atan2(wy-mapPose.y, wx-mapPose.x) : 0;
  gbSay('끌어서 방향을 정하고 놓으면 출발합니다');
  drawMap();
}
function goalDrag(cv,e){
  const r=cv.getBoundingClientRect();
  const [wx,wy]=s2w(cv, e.clientX-r.left, e.clientY-r.top);
  const dx=wx-goal.x, dy=wy-goal.y;
  // 조금만 움직인 것은 손떨림으로 본다. 그때는 기본 방향을 그대로 둔다.
  if(Math.hypot(dx,dy)>0.08) goal.yaw=Math.atan2(dy,dx);
  drawMap();
}
async function goalUp(){
  if(!goal.has || !goal.setting) return;
  // 몸짓 받기는 여기서 끝낸다. 열어 둔 채로 지도를 만지다가 실수로
  // 또 보내는 일을 막는다. 다시 보내려면 '이동' 을 한 번 더 누른다.
  goal.setting=false;
  // 막대는 남겨 둔다. 이동하는 동안 상태와 정지 버튼이 보여야 한다.
  await goalSend();
  drawMap();
}

// ⚠ 2026-11-13 끈 만큼만 움직인다(상대). 예전에는 누른 자리로 로봇이 뛰어가고
//   (위치), 로봇 중심에서 누른 곳 방향을 그대로 써서(회전) 손가락이 로봇을 가리고
//   가까이서는 각도가 튀어 미세하게 못 맞췄다(사용자: "미세 이동이 안됨").
//   회전은 화면 가로로 끈 거리 1px 당 0.1도.
function relocDrag(cv,e,start){
  reloc.src='manual';
  const r=cv.getBoundingClientRect();
  const [wx,wy]=s2w(cv, e.clientX-r.left, e.clientY-r.top);
  if(start){ reloc.s={wx, wy, px:e.clientX, x:reloc.x, y:reloc.y, yaw:reloc.yaw}; return; }
  const st=reloc.s; if(!st) return;
  if(reloc.mode==='rot'){
    reloc.yaw = st.yaw - (e.clientX-st.px)*Math.PI/1800;
  }else{
    reloc.x = st.x + (wx-st.wx); reloc.y = st.y + (wy-st.wy);
  }
  relocShow(); drawMap();
}
function relocShow(){
  rbSay('('+reloc.x.toFixed(3)+', '+reloc.y.toFixed(3)+') '
        +(reloc.yaw*180/Math.PI).toFixed(1)+'° — 확인을 눌러 적용');
}
// 미세 버튼: 로봇 기준 앞(+)/왼쪽(+) 한 칸, 왼쪽으로 돌기(+). 작게 1cm·0.5°, 크게 5cm·2°
let relocBig=false;
function relocStep(b){
  relocBig=!relocBig;
  document.querySelectorAll('.relocbar .nud [data-big]').forEach(x=>
    x.textContent = relocBig ? '5cm·2°' : '1cm·0.5°');
}
function relocNudge(f,l,r){
  if(!reloc.setting) return;
  reloc.src='manual';
  const d = relocBig ? 0.05 : 0.01, a = (relocBig ? 2 : 0.5)*Math.PI/180;
  const c=Math.cos(reloc.yaw), s=Math.sin(reloc.yaw);
  reloc.x += f*d*c - l*d*s; reloc.y += f*d*s + l*d*c; reloc.yaw += r*a;
  relocShow(); drawMap();
}

// 네트워크 설정.
//   현장에 나가면 공유기가 바뀐다. 그때마다 로봇에 모니터를 붙이지 않고
//   여기서 바꾼다. 바꾸는 순간 이 화면과의 연결이 끊기므로, 무엇이 일어날지
//   미리 적어 두고 실패하면 서버가 알아서 되돌린다.
let nwIpMode='auto';
// 핫스팟 <-> WiFi. 무선 한 장이라 둘 중 하나만 쓴다.
// 어느 쪽으로 바꾸든 지금 보고 있는 연결이 끊기므로, 새 주소를 먼저 알린다.
async function nwSetMode(m){
  const msg=$('nwModeMsg');
  const to = m==='ap' ? '핫스팟' : 'WiFi';
  msg.innerHTML='<b>'+to+'</b> 로 바꾸는 중... 이 화면과의 연결이 끊깁니다.<br>'+
    (m==='ap' ? '핫스팟에 붙어 http://10.42.0.1:8090 으로 들어오세요.'
              : 'WiFi 쪽 주소로 다시 들어오세요. 붙지 못하면 핫스팟으로 되돌립니다.');
  msg.style.color='var(--warn)';
  try{
    const r=await api('/network/mode',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({mode:m})});
    msg.textContent=r.message; msg.style.color=r.ok?'var(--ok)':'var(--bad)';
    if(r.ok && r.address){
      msg.innerHTML=r.message+'<br><a href="http://'+r.address+
        ':8090" style="color:var(--ok)">http://'+r.address+':8090 으로 이동</a>';
    }
    nwNow();
  }catch(e){
    msg.innerHTML='응답이 끊겼습니다 — 모드가 바뀌면 정상입니다.<br>'+
      (m==='ap' ? '핫스팟에 붙어 http://10.42.0.1:8090 으로 들어오세요.'
                : 'WiFi 쪽 새 주소로 들어와 보세요.');
    msg.style.color='var(--warn)';
  }
}
async function nwApSave(){
  const msg=$('nwModeMsg');
  const body={};
  const ssid=$('nwApSsid').value.trim(), pw=$('nwApPw').value;
  if(ssid) body.ssid=ssid;
  if(pw) body.password=pw;
  if(!ssid && !pw){ msg.textContent='바꿀 값을 넣으세요';
                    msg.style.color='var(--bad)'; return; }
  msg.textContent='저장 중...'; msg.style.color='var(--dim)';
  try{
    const r=await api('/network/ap',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    msg.textContent=r.message; msg.style.color=r.ok?'var(--ok)':'var(--bad)';
    $('nwApPw').value=''; nwNow();
  }catch(e){ msg.textContent='요청 실패'; msg.style.color='var(--bad)'; }
}

async function nwNow(){
  try{
    const d=await api('/network');
    const ap=d.ap||{};
    $('nwTogAp').className   = ap.mode==='ap'   ? 'sel' : '';
    $('nwTogWifi').className = ap.mode==='wifi' ? 'sel' : '';
    if(!$('nwApSsid').value && ap.ssid) $('nwApSsid').value=ap.ssid;
    $('nwApInfo').innerHTML=
      row('지금 모드', ap.mode==='ap'
            ? '<span class="ok-txt">핫스팟</span>'
            : ap.mode==='wifi' ? '<span class="ok-txt">WiFi</span>'
            : '<span class="bad-txt">연결 없음</span>')+
      row('핫스팟 이름', ap.ssid||'-')+
      row('핫스팟 암호', ap.has_password?'설정됨':'<span class="bad-txt">없음(공개)</span>')+
      (ap.mode==='ap'
        ? row('접속 주소','<b>http://'+(ap.address||'10.42.0.1')+':8090</b>')+
          row('붙어 있는 기기', (ap.clients||0)+' 대')
        : '');
    $('nwNow').innerHTML=(d.devices||[]).map(v=>{
      // nmcli 는 자리에 따라 'connected' 로도 '100 (연결됨)' 으로도 준다
      const st = /connected|100/.test(v.state||'') &&
                 !/disconnected/.test(v.state||'') ? 'ok-txt' : 'dim-txt';
      return row(v.device+' <span class="dim-txt">'+v.type+'</span>',
        '<span class="'+st+'">'+(v.ssid||v.connection||'연결 없음')+'</span>'+
        (v.ip?'<br><span class="dim-txt">'+v.ip+
             (v.gateway?' → '+v.gateway:'')+
             (v.method?' · '+(v.method==='auto'?'DHCP':'고정'):'')+'</span>':''));
    }).join('');
    $('nwHint').textContent=d.hint||'';
    $('nwHint').style.color='var(--bad)';
  }catch(e){ $('nwNow').innerHTML='<tr><td class="bad-txt">불러오지 못했습니다</td></tr>'; }
}
async function nwList(rescan){
  const b=$('nwScan'); b.disabled=true;
  b.textContent = rescan?'찾는 중... (몇 초)':'주변 WiFi 다시 찾기';
  try{
    const d=await api('/network/wifi'+(rescan?'':'?rescan=0'));
    $('nwAps').innerHTML=(d.networks||[]).length
      ? d.networks.map(n=>
          '<tr><td>'+(n.active?'<span class="ok-txt">● </span>':'')+
          '<b>'+esc(n.ssid)+'</b> <span class="dim-txt">'+n.security+
          ' · '+n.freq+'</span></td><td style="text-align:right">'+
          n.signal+'% <button class="rb" style="margin-left:8px;padding:3px 9px;'+
          'font-size:12px" onclick="nwPick('+JSON.stringify(n.ssid)
             .replace(/"/g,'&quot;')+',&quot;'+n.security+'&quot;)">선택</button></td></tr>').join('')
      : '<tr><td class="dim-txt">'+(d.message||'찾은 WiFi 가 없습니다')+'</td></tr>';
  }catch(e){ $('nwAps').innerHTML='<tr><td class="bad-txt">목록을 못 읽었습니다</td></tr>'; }
  b.disabled=false; b.textContent='주변 WiFi 다시 찾기';
}
function esc(t){ return String(t).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c])); }
function nwPick(ssid, sec){
  $('nwPick').style.display='';
  $('nwSsid').value=ssid;
  $('nwSec').textContent=sec||'';
  $('nwPw').value=''; $('nwPw').focus();
  nwMode('auto');
}
function nwMode(m){
  nwIpMode=m;
  $('nwMAuto').className = m==='auto'?'sel':'';
  $('nwMMan').className  = m==='manual'?'sel':'';
  $('nwManual').style.display = m==='manual'?'':'none';
}
async function nwConnect(){
  const msg=$('nwMsg'), btn=$('nwGo');
  const ssid=$('nwSsid').value;
  const body={ssid:ssid, password:$('nwPw').value};
  if(nwIpMode==='manual'){
    body.ip={mode:'manual', address:$('nwIp').value.trim(),
             prefix:parseInt($('nwPrefix').value)||24,
             gateway:$('nwGw').value.trim(), dns:$('nwDns').value.trim()};
  }else{ body.ip={mode:'auto'}; }
  btn.disabled=true;
  msg.innerHTML='적용 중... 이 화면과의 연결이 끊길 수 있습니다.<br>'+
    '붙지 못하면 원래 연결로 자동 복구됩니다 (최대 1분 30초).';
  msg.style.color='var(--dim)';
  try{
    const r=await api('/network/wifi/connect',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    msg.textContent=r.message; msg.style.color=r.ok?'var(--ok)':'var(--bad)';
    if(r.ok && r.ip){
      const host=r.ip.split('/')[0];
      msg.innerHTML=r.message+'<br><a href="http://'+host+':8090" '+
        'style="color:var(--ok)">http://'+host+':8090 으로 이동</a>';
    }
    nwNow();
  }catch(e){
    // 응답이 안 오는 것은 정상일 수 있다 - 주소가 바뀌면 이 연결은 끊긴다
    msg.innerHTML='응답이 끊겼습니다. 주소가 바뀌었을 수 있습니다.<br>'+
      '새 주소로 다시 들어와 보세요. 붙지 못했다면 로봇이 원래 연결로 '+
      '돌아와 있습니다.';
    msg.style.color='var(--warn)';
  }
  btn.disabled=false;
}

// 새 대차 규격 재기.
//   carts.yaml 에 넣는 값은 도면 치수가 아니라 라이다가 보는 다리 중심 간
//   거리다. 도면값을 그대로 넣었다가 95mm 어긋나 인식률이 11% 로 깜빡인
//   적이 있어(2026-08-28 S300), 재서 넣도록 화면을 뒀다.
let ccRes=null;
async function ccLoad(){
  const sv=$('ccSecV'), sl=$('ccSec');
  if(sl&&sv){ sv.textContent=sl.value+' 초';
              sl.oninput=()=>{ sv.textContent=sl.value+' 초'; }; }
  try{
    const d=await api('/carts/specs');
    $('ccList').innerHTML=(d.carts||[]).length
      ? (d.carts||[]).map(c=>
          '<tr><td>'+c.name+'</td><td style="text-align:right">'+
          (c.width*1000).toFixed(0)+' x '+(c.depth*1000).toFixed(0)+' mm'+
          ' <span class="dim-txt">±'+(c.tol*1000).toFixed(0)+'mm</span>'+
          ' <button class="rb" style="margin-left:8px;padding:3px 9px;font-size:12px"'+
          ' onclick="ccDrop(\''+c.name+'\')">삭제</button></td></tr>').join('')
      : '<tr><td class="dim-txt">등록된 대차가 없습니다</td></tr>';
    $('ccFile').textContent = d.file
      ? ('보정 반지름 '+((d.leg_radius||0)*1000).toFixed(0)+' mm · '+d.file) : '';
  }catch(e){ $('ccList').innerHTML='<tr><td class="bad-txt">불러오지 못했습니다</td></tr>'; }
}
async function ccMeasure(){
  const btn=$('ccGo'), msg=$('ccMsg');
  const sec=parseFloat($('ccSec').value)||6;
  ccRes=null; $('ccReg').style.display='none'; $('ccOut').innerHTML='';
  btn.disabled=true;
  msg.textContent='재는 중... ('+sec+'초, 로봇과 대차를 움직이지 마세요)';
  msg.style.color='var(--dim)';
  try{
    const r=await api('/carts/measure',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({seconds:sec, front:$('ccFront').checked,
                           name:($('ccName').value||'NEW').trim()})});
    msg.textContent=r.message; msg.style.color=r.ok?'var(--ok)':'var(--bad)';
    const v=r.result;
    if(v && r.ok){
      ccRes=v;
      const mm=x=>(x*1000).toFixed(1)+' mm';
      $('ccOut').innerHTML=
        row('가로 (width)','<b>'+mm(v.width)+'</b> <span class="dim-txt">±'+
            (v.spread_w*1000).toFixed(1)+' · '+mm(v.range_w[0])+'~'+mm(v.range_w[1])+'</span>')+
        row('세로 (depth)','<b>'+mm(v.depth)+'</b> <span class="dim-txt">±'+
            (v.spread_d*1000).toFixed(1)+' · '+mm(v.range_d[0])+'~'+mm(v.range_d[1])+'</span>')+
        row('허용 오차 (tol)',mm(v.tol))+
        row('쓸 만한 프레임',v.valid+' / '+v.frames)+
        row('다리 후보',v.candidates+' 개')+
        row('대차까지',v.dist.toFixed(2)+' m, '+v.yaw_deg.toFixed(0)+'°')+
        row('흔들림', v.shaky
            ? '<span class="bad-txt">큽니다 — 다시 재기를 권합니다</span>'
            : '<span class="ok-txt">안정적</span>');
      $('ccReg').style.display='';
    }else if(v){
      $('ccOut').innerHTML=row('관찰한 프레임',v.frames)+
        row('사각형을 찾은 프레임',v.valid)+row('다리 후보',v.candidates+' 개');
    }
  }catch(e){ msg.textContent='요청 실패'; msg.style.color='var(--bad)'; }
  btn.disabled=false;
}
async function ccDrop(name){
  const msg=$('ccMsg');
  msg.textContent="'"+name+"' 을 목록에서 뺍니다..."; msg.style.color='var(--dim)';
  try{
    const r=await api('/carts/unregister',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({name:name})});
    msg.textContent=r.message; msg.style.color=r.ok?'var(--ok)':'var(--bad)';
    if(r.ok) setTimeout(ccLoad,1500);
  }catch(e){ msg.textContent='요청 실패'; msg.style.color='var(--bad)'; }
}
async function ccRegister(){
  if(!ccRes) return;
  const nm=($('ccName').value||'').trim();
  const msg=$('ccMsg');
  if(!nm){ msg.textContent='이름을 넣어 주세요'; msg.style.color='var(--bad)'; return; }
  msg.textContent='등록 중...'; msg.style.color='var(--dim)';
  try{
    const r=await api('/carts/register',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({name:nm, width:ccRes.width, depth:ccRes.depth,
                           tol:ccRes.tol})});
    msg.textContent=r.message; msg.style.color=r.ok?'var(--ok)':'var(--bad)';
    if(r.ok){ $('ccReg').style.display='none'; setTimeout(ccLoad,1500); }
  }catch(e){ msg.textContent='요청 실패'; msg.style.color='var(--bad)'; }
}

// 이어서 매핑. 지금 서 있는 자리를 시작 자세로 넘겨 기존 지도에 덧그린다.
async function mapResume(){
  const el=$('mapMsg');
  if(el){ el.textContent='이어서 매핑 준비 중...'; el.style.color='var(--dim)'; }
  try{
    const r=await api('/mapping/resume',{method:'POST'});
    if(el){ el.textContent=r.message; el.style.color=r.ok?'var(--ok)':'var(--bad)'; }
    if(r.ok) clearTrail();
    loadMapping(); refreshMap();
  }catch(e){ if(el){ el.textContent='요청 실패'; el.style.color='var(--bad)'; } }
}

// 지도 수정 (지우개).
//   매핑할 때는 있었지만 지금은 치워진 물건이 지도에 검게 남는다. 그 자리를
//   지워 다시 갈 수 있는 곳으로 되돌린다.
//   위치 재조정과 같은 방식으로, 확인을 누르기 전에는 아무것도 반영되지 않는다.
//
//   saved  = 서버에 이미 적용돼 있는 지운 자리
//   pend   = 지금 화면에서 새로 칠한 것. 획 단위로 묶어 두어 되돌리기가
//            한 획씩 물러난다 (원 하나씩 물러나면 수십 번 눌러야 한다)
let edit={on:false, mode:'erase', brush:0.25, saved:[], pend:[], pts:[],
          drawing:false};

// 세 가지 모드.
//   지우개   = 치워진 물건을 지워 다시 갈 수 있는 곳으로 (끌어서 칠함)
//   가상벽   = 로봇이 넘지 말아야 할 선   (점을 찍어 만든다)
//   금지구역 = 로봇이 들어가지 말 영역     (점을 찍어 만든다)
//
// 선과 영역은 브러시가 아니라 점으로 만든다. 원을 줄줄이 찍으면 선 하나에
// 수십 개가 들어가고 가장자리가 울퉁불퉁하며 나중에 고치기 어렵다.
//
// 가상벽·금지구역은 위치추정 지도에 안 들어간다 - 라이다가 볼 수 없는 벽을
// 지도에 넣으면 '여기서 벽이 보여야 한다' 고 기대해 위치추정이 나빠진다.
const EDIT_KIND={erase:'circle', wall:'line', zone:'poly'};
function editOp(m){
  if(edit.pts.length) editShapeDone();      // 그리던 것은 마무리
  edit.mode=m; edit.pts=[];
  $('ebErase').className='rb'+(m==='erase'?' sel':'');
  $('ebWall').className ='rb'+(m==='wall' ?' sel':'');
  $('ebZone').className ='rb'+(m==='zone' ?' sel':'');
  $('ebDone').style.display = m==='erase' ? 'none' : '';
  ebSay(m==='erase' ? '지도를 끌어 지울 자리를 칠하세요'
      : m==='wall'  ? '지도를 눌러 점을 찍으세요 — 이은 선이 가상벽이 됩니다 (완료로 마무리)'
                    : '지도를 눌러 모서리를 찍으세요 — 감싼 안쪽이 금지구역이 됩니다 (완료로 마무리)');
  drawMap();
}
// 찍던 점들을 하나의 도형으로 확정한다
function editShapeDone(){
  const need = edit.mode==='wall' ? 2 : 3;
  if(edit.pts.length < need){
    if(edit.pts.length){ ebSay('점이 '+need+'개는 있어야 합니다'); edit.pts=[]; drawMap(); }
    return;
  }
  const sh={op:'wall', kind:EDIT_KIND[edit.mode], points:edit.pts.slice()};
  if(edit.mode==='wall') sh.width=+edit.brush.toFixed(4);
  edit.pend.push([sh]);
  edit.pts=[];
  ebSay((edit.mode==='wall'?'가상벽':'금지구역')+' 1개를 만들었습니다 — '+
        '확인을 눌러야 반영됩니다');
  drawMap();
}

function editOpen(){
  if(edit.on){ editClose(); return; }
  if(reloc.setting) relocCancel();
  edit.on=true; edit.pend=[]; edit.drawing=false;
  const b=$('editBar'); if(b) b.classList.add('on');
  const t=$('btnEdit'); if(t) t.textContent='수정 끝내기';
  editOp('erase');
  ebSay('불러오는 중...');
  editLoad();
  drawMap();
}
function editClose(){
  edit.on=false; edit.pend=[]; edit.pts=[]; edit.drawing=false;
  const b=$('editBar'); if(b) b.classList.remove('on');
  const t=$('btnEdit'); if(t) t.textContent='지도 수정';
  drawMap();
}
function ebSay(msg, kind){
  const e=$('ebMsg'); if(!e) return;
  e.textContent=msg||'';
  e.style.color = kind==='ok' ? 'var(--ok)' : kind==='bad' ? 'var(--bad)' : 'var(--dim)';
}
async function editLoad(){
  try{
    const r=await api('/map/edits');
    edit.saved = (r.shapes||[]);
    if(!r.editable){
      ebSay(r.map ? '매핑 중에는 수정할 수 없습니다'
                  : '적용된 지도가 없습니다 — 맵 화면에서 지도를 적용하세요','bad');
    }else{
      const ne=edit.saved.filter(x=>(x.op||'erase')==='erase').length;
      const nl=edit.saved.filter(x=>x.op==='wall'&&x.kind==='line').length;
      const nz=edit.saved.filter(x=>x.op==='wall'&&x.kind==='poly').length;
      ebSay(edit.saved.length
        ? ('적용됨 — 지운 자리 '+ne+' 곳 · 가상벽 '+nl+' 개 · 금지구역 '+nz+' 개')
        : '지도를 끌어 지울 자리를 칠하세요');
    }
    drawMap();
  }catch(e){ ebSay('불러오지 못했습니다','bad'); }
}
function editBrush(v){ edit.brush=parseFloat(v)||0.25; }

// 획 하나 = 원 여러 개. 끌면서 브러시 반경의 절반마다 원을 하나씩 놓는다.
// 매 픽셀마다 놓으면 도형이 수천 개가 되어 저장도 그리기도 무거워진다.
function editStroke(cv,e,startNew){
  const r=cv.getBoundingClientRect();
  const [wx,wy]=s2w(cv, e.clientX-r.left, e.clientY-r.top);
  if(startNew) edit.pend.push([]);
  const st=edit.pend[edit.pend.length-1];
  if(!st) return;
  const last=st[st.length-1];
  if(last && Math.hypot(wx-last.x, wy-last.y) < edit.brush*0.5) return;
  st.push({op:'erase', kind:'circle', x:+wx.toFixed(4), y:+wy.toFixed(4),
           r:+edit.brush.toFixed(4)});
  ebSay('지울 자리 '+(edit.saved.length+editPendCount())+
        ' 곳 — 확인을 눌러야 반영됩니다');
  drawMap();
}
function editPendCount(){ return edit.pend.reduce((a,st)=>a+st.length,0); }

function editUndo(){
  // 찍던 점부터, 그다음 새로 만든 것, 마지막에 이미 적용된 것 순으로 무른다.
  if(edit.pts.length){ edit.pts.pop();
    ebSay('점 '+edit.pts.length+'개'); drawMap(); return; }
  if(edit.pend.length){ edit.pend.pop(); }
  else if(edit.saved.length){ edit.saved.pop(); }
  else { ebSay('되돌릴 것이 없습니다'); return; }
  ebSay('되돌렸습니다 — 확인을 눌러야 반영됩니다');
  drawMap();
}
function editReset(){
  edit.pts=[];
  // 아직 확인 안 누른 것만 있으면 그냥 버린다. 되돌릴 게 없기 때문이다.
  if(!edit.saved.length){
    edit.pend=[];
    ebSay('칠한 것을 버렸습니다');
    drawMap(); return;
  }
  // 이미 적용된 것까지 지우는 일은 되돌릴 수 없으므로 한 번 묻는다.
  $('edResetN').textContent = edit.saved.length+'곳';
  $('edResetBg').classList.add('on');
}
function editResetClose(){ $('edResetBg').classList.remove('on'); }
async function editResetGo(){
  // 창에서 '전부 지우기' 를 골랐으면 여기서 바로 반영한다. 툴바의 확인을
  // 또 누르게 하면 무엇이 실제로 지워진 것인지 헷갈린다.
  editResetClose();
  edit.saved=[]; edit.pend=[]; edit.pts=[]; drawMap();
  ebSay('지우는 중...');
  try{
    const r=await api('/map/edits',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({shapes:[]})});
    ebSay(r.message, r.ok?'ok':'bad');
    if(r.ok) refreshMap();
  }catch(e){ ebSay('요청 실패','bad'); }
}
async function editConfirm(){
  if(edit.pts.length) editShapeDone();     // 그리던 것 마무리
  const shapes=edit.saved.concat(...edit.pend);
  ebSay('적용 중...');
  try{
    const r=await api('/map/edits',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({shapes:shapes})});
    ebSay(r.message, r.ok?'ok':'bad');
    if(r.ok){
      edit.saved=shapes; edit.pend=[];
      refreshMap();          // 서버가 합성한 지도를 다시 받아 온다
      drawMap();
    }
  }catch(e){ ebSay('요청 실패','bad'); }
}

// 장치 재시작/리셋. 응답이 올 때까지 수십 초 걸릴 수 있어 버튼을 잠근다.
async function devAct(url, label){
  const el = url.indexOf('lidar')>=0 ? $('devMsgL') : $('devMsgC');
  const btns=document.querySelectorAll('#pane-'+cur+' .act');
  btns.forEach(b=>b.disabled=true);
  if(el){ el.textContent=label+' 재시작 중... (최대 30초)'; el.style.color='var(--dim)'; }
  try{
    const r=await api(url,{method:'POST'});
    if(el){ el.textContent=r.message; el.style.color=r.ok?'var(--ok)':'var(--bad)'; }
  }catch(e){ if(el){ el.textContent='요청 실패'; el.style.color='var(--bad)'; } }
  btns.forEach(b=>b.disabled=false);
}

function bindMapDrag(){
  const cv=mapCanvas(); if(!cv||cv._bound) return; cv._bound=true;
  cv.addEventListener('pointerdown',e=>{
    if(edit.on){
      if(edit.mode!=='erase'){          // 선·영역은 끌지 않고 점을 찍는다
        const r=cv.getBoundingClientRect();
        const [wx,wy]=s2w(cv, e.clientX-r.left, e.clientY-r.top);
        edit.pts.push([+wx.toFixed(4), +wy.toFixed(4)]);
        ebSay('점 '+edit.pts.length+'개 — 완료를 누르면 만들어집니다');
        drawMap(); return;
      }
      edit.drawing=true; cv.setPointerCapture(e.pointerId);
      editStroke(cv,e,true); return;
    }
    if(goal.setting){
      goal.dragging=true; cv.setPointerCapture(e.pointerId);
      goalDown(cv,e); return;
    }
    if(reloc.setting){
      reloc.dragging=true; cv.setPointerCapture(e.pointerId);
      relocDrag(cv,e,true); return;
    }
    const r0=cv.getBoundingClientRect();
    const k0=cv.width/r0.width;
    view.hitRobot = hitRobot(cv,(e.clientX-r0.left)*k0,(e.clientY-r0.top)*k0);
    view.downT=Date.now(); view.downX=e.clientX; view.downY=e.clientY;
    view.dragging=true; view.lx=e.clientX; view.ly=e.clientY;
    cv.classList.add('drag'); cv.setPointerCapture(e.pointerId);
    const tf=mapTg('tgFollow'); if(tf) tf.checked=false;
  });
  cv.addEventListener('pointermove',e=>{
    if(edit.drawing){ editStroke(cv,e,false); return; }
    if(goal.dragging){ goalDrag(cv,e); return; }
    if(reloc.dragging){ relocDrag(cv,e); return; }
    if(!view.dragging) return;
    const k=cv.width/cv.getBoundingClientRect().width;
    view.cx -= (e.clientX-view.lx)*k/view.ppm;
    view.cy += (e.clientY-view.ly)*k/view.ppm;
    view.lx=e.clientX; view.ly=e.clientY; drawMap();
  });
  // ⚠ pointercancel 은 브라우저가 터치를 거둬 간 것(스크롤·제스처·전화 등)이지
  //   사람이 손을 뗀 것이 아니다. 예전에는 둘 다 같은 end() 라 취소돼도
  //   이동 목표가 전송됐다(2026-09-23 관리자웹 검토 P1). 취소면 보내지 않는다.
  const end=(e)=>{
    const cancel = !!e && e.type==='pointercancel';
    if(edit.drawing){ edit.drawing=false; return; }
    if(goal.dragging){
      goal.dragging=false;
      if(cancel){ goal.has=false;
        gbSay('터치가 취소되어 보내지 않았습니다. 다시 눌러 주세요','bad');
        drawMap(); return; }
      goalUp(); return;
    }
    if(reloc.dragging){ reloc.dragging=false; return; }
    if(view.dragging) saveUI();
    // 끌지 않고 로봇을 짧게 눌렀으면 재조정 창을 연다
    if(!cancel && view.dragging && view.hitRobot &&
       Date.now()-view.downT < 400) relocOpen();
    view.hitRobot=false;
    view.dragging=false; cv.classList.remove('drag');
  };
  cv.addEventListener('pointerup',end); cv.addEventListener('pointercancel',end);
  cv.addEventListener('wheel',e=>{ e.preventDefault();
    zoomBy(e.deltaY<0?1.12:0.89); },{passive:false});
}

async function refreshMap(){
  if(cur!=='dash' && cur!=='mapping') return;
  // 경로는 지도와 같은 주기로 받는다. 계획은 1Hz 로 갱신되므로 충분하다.
  try{
    const pth=await api('/nav/path');
    navPath = (pth && pth.points) ? pth.points : [];
  }catch(e){ navPath=[]; }
  try{
    const m=await api('/map/info');
    const had = mapMeta && mapMeta.available;
    // 지도가 사라졌다가 새로 생기는 순간이 곧 '다른 지도' 가 된 시점이다
    // (새 매핑 시작, 저장 후 적용, 지도 교체). 그때 궤적을 버린다.
    // 매핑 중에는 지도가 계속 커지지만 available 은 참으로 유지되므로
    // 크기 변화로 판별하면 안 된다.
    if(had !== !!m.available) clearTrail();
    mapMeta=m;
    if(m.available && m.seq!==mapSeq){
      mapSeq=m.seq;
      const img=new Image();
      // 저장된 화면 위치가 있으면 자동 맞춤으로 덮어쓰지 않는다
      img.onload=()=>{ mapImg=img; if(!had && !viewRestored) resetView();
                       drawMap(); };
      img.src='/map.png?'+m.seq;
    }
    $('dot-maps').className='dot '+(m.available?'on':'off');
    const h=cur==='mapping' ? $('mapHint2') : $('mapHint');
    if(h) h.innerHTML = m.available
      ? '맵 '+m.width+'×'+m.height+' px, '+m.resolution.toFixed(3)+' m/px — '
        +'드래그로 이동, 휠로 확대'
      : '<b style="color:var(--warn)">맵 없음</b> — SLAM 이 실행 중이어야 합니다. '
        +'<code>ros2 launch und_amr_bringup slam_test.launch.py</code> '
        +'(정지 검증) 또는 <code>slam.launch.py</code> (주행)';
  }catch(e){}
}

// ---------------------------------------------------------------- 그리기
function draw(){
  if(cur==='dash'||cur==='mapping'){ drawMap(); return; }
  const cv = cur==='lidar' ? $('cvLidar') : null;
  if(!cv || !cv.getContext) return;
  const g=cv.getContext('2d');
  const rEl=$('rng');
  const R = (cur==='lidar'&&rEl) ? +rEl.value : 8;
  if(rEl) $('rngV').textContent=R+' m';
  const W=cv.width,H=cv.height,cx=W/2,cy=H/2,sc=(Math.min(W,H)/2-24)/R;
  g.fillStyle='#080a0e'; g.fillRect(0,0,W,H);
  g.strokeStyle='#1c2230'; g.fillStyle='#48505f'; g.font='11px ui-monospace,monospace';
  const st=R<=5?1:R<=12?2:R<=25?5:10;
  for(let r=st;r<=R;r+=st){ g.beginPath(); g.arc(cx,cy,r*sc,0,7); g.stroke();
    g.fillText(r+'m',cx+4,cy-r*sc-3); }
  g.beginPath(); g.moveTo(cx,0);g.lineTo(cx,H);g.moveTo(0,cy);g.lineTo(W,cy); g.stroke();
  if(scan&&scan.points) for(const [ad,d] of scan.points){
    const a=(ad-90)*Math.PI/180, x=cx+Math.cos(a)*d*sc, y=cy+Math.sin(a)*d*sc;
    if(x<0||y<0||x>W||y>H) continue;
    g.fillStyle = d<0.5?'#ff5c5c' : d<1.0?'#ffb454' : '#4da3ff';
    g.fillRect(x-1.5,y-1.5,3,3);
  }
  g.fillStyle='#3ddc97'; g.beginPath();
  g.moveTo(cx,cy-11);g.lineTo(cx-8,cy+8);g.lineTo(cx+8,cy+8);g.closePath();g.fill();
}

// ---------------------------------------------------------------- 로딩
// 화면을 옮기면 패널이 새로 그려져 img 의 src 가 사라진다. 플래그 대신 src 로 본다.
function camSrc(id, url){ const e=$(id); if(e && !(e.getAttribute('src')||'').includes('.mjpg'))
  e.src=url+'?'+Date.now(); }
function startCam(){ camOn=true;
  camSrc('imgCam','/camera/color.mjpg'); camSrc('imgCamDn2','/camera/down.mjpg'); }
async function mapStart(){
  if(slamOn && !confirm('작성 중인 지도를 비우고 처음부터 새로 시작합니다. 계속할까요?'))
    return;
  // 지도를 비우면 궤적도 함께 비운다. 이전 궤적은 사라진 지도의 좌표라
  // 그대로 두면 새 지도 위에 엉뚱한 선이 이어져 그려진다.
  clearTrail();
  await mapAct('/mapping/start');
}
function clearTrail(){ trail=[]; mapPose=null; }
function toggleCart(){
  cartOn = !cartOn;
  syncCartBtn();
  if(!cartOn){ carts=null; cartsErr=null; }
  drawMap(); saveUI();
}
// 매핑 종료는 "저장할까?" -> "이름은?" 순으로 묻는다.
// 어느 단계에서든 '저장 안 하고 종료' 를 고르면 그 매핑은 버린다.
function mapStop(){
  // 이어서 매핑한 경우에는 이름을 묻지 않는다. 원래 지도를 넓히려고 이어
  // 그린 것이므로 그 지도에 덮어쓰는 것이 기본이다.
  const res = !!resumedFrom;
  $('stopStep0').style.display = res ? '' : 'none';
  $('stopStep1').style.display = res ? 'none' : '';
  $('stopStep2').style.display='none';
  $('stopMsg').textContent='';
  if(res) $('stopBase').textContent = "'"+resumedFrom+"'";
  const n=$('stopName');
  if(n){
    const d=new Date(), z=v=>String(v).padStart(2,'0');
    n.value = resumedFrom ||
      ('map_'+z(d.getMonth()+1)+z(d.getDate())+'_'+z(d.getHours())+z(d.getMinutes()));
  }
  $('stopBg').classList.add('on');
}
function stopClose(){ $('stopBg').classList.remove('on'); }
function stopGoName(){
  $('stopStep0').style.display='none';
  $('stopStep1').style.display='none';
  $('stopStep2').style.display='';
  const n=$('stopName'); if(n){ n.focus(); n.select(); }
}
function stopSaveBase(){
  if(!resumedFrom) return stopGoName();
  return stopSaveAs(resumedFrom);
}
async function stopSaveGo(){ return stopSaveAs(($('stopName').value||'').trim()); }
async function stopSaveAs(nm){
  if(!nm){ $('stopMsg').textContent='이름을 넣어 주세요';
           $('stopMsg').style.color='var(--bad)'; return; }
  $('stopMsg').textContent='저장 중...'; $('stopMsg').style.color='var(--dim)';
  const ok = await mapAct('/mapping/save', {name: nm});
  if(ok===false){ $('stopMsg').textContent='저장에 실패해 종료하지 않았습니다';
                  $('stopMsg').style.color='var(--bad)'; return; }
  await mapAct('/mapping/stop');
  stopClose(); loadMaps();
}
async function stopDiscard(){
  $('stopMsg').textContent='종료 중...'; $('stopMsg').style.color='var(--dim)';
  await mapAct('/mapping/stop');
  stopClose();
}
async function mapPause(){ await mapAct('/mapping/pause'); }
async function mapClear(){ await mapAct('/mapping/clear'); }
async function mapAct(url, body){
  const el=$('mapMsg'); if(el){ el.textContent='처리 중...'; el.style.color='var(--dim)'; }
  try{
    const r=await api(url, body
      ? {method:'POST',headers:{'Content-Type':'application/json'},
         body:JSON.stringify(body)}
      : {method:'POST'});
    if(el){ el.textContent=r.message||''; 
            el.style.color = r.ok ? 'var(--ok)' : 'var(--bad)'; }
    loadMapping();
    return !!r.ok;
  }catch(e){ if(el){ el.textContent='요청 실패'; el.style.color='var(--bad)'; }
             loadMapping(); return false; }
}

async function loadMapping(){
  try{
    const d=await api('/mapping/status');
    const m=d.map||{};
    $('tMapping').innerHTML =
      row('SLAM', d.slam_running
          ? '<span class="ok-txt">지도 작성 중</span>'
          : d.localizing
            ? '<span class="ok-txt">위치추정 중</span> <span class="dim-txt">(지도 안 늘어남)</span>'
            : '<span class="bad-txt">미실행</span>')+
      row('스캔 반영', d.paused
          ? '<span class="warn-txt">일시정지</span>' : '진행 중')+
      row('지도 크기', m.available ? m.width+' x '+m.height+' px' : '-')+
      row('해상도', m.available ? m.resolution.toFixed(3)+' m/px' : '-')+
      row('실제 면적', m.available
          ? (m.width*m.resolution).toFixed(1)+' x '+(m.height*m.resolution).toFixed(1)+' m'
          : '-')+
      row('저장된 지도', (d.saved_maps||[]).length+' 개');
    slamOn = !!d.slam_running;
    resumedFrom = d.resumed_from || null;
    const bp=$('btnPause'); if(bp){ bp.textContent=d.paused?'재개':'일시정지';
                                    bp.disabled=!slamOn; }
    // 종료는 '지도를 그리는 중' 일 때만. 매핑을 끝내면 위치추정 SLAM 이
    // 새로 뜨는데, owned_by_api 만 보면 그것도 우리 것이라 계속 활성화된다.
    const bs=$('btnStop');  if(bs) bs.disabled = !(d.slam_running && d.owned_by_api);
    const bc=$('btnStart'); if(bc) bc.disabled = false;
    const bt=$('btnStart'); if(bt) bt.textContent='새 매핑 시작';
    // 이어서 매핑 — 못 누를 때는 왜 못 누르는지 그 자리에 적는다.
    // 회색으로만 두면 고장으로 읽힌다.
    const br=$('btnResume'); if(br) br.disabled = !d.can_resume;
    // 지도 수정은 저장된 지도 위에서만 한다. 매핑 중인 지도는 이름이 없어
    // 편집을 붙일 데가 없다(서버도 거절한다). 이유를 붙여 잠근다.
    const be=$('btnEdit');
    if(be){
      be.disabled = slamOn || !d.active_map;
      be.title = slamOn ? '매핑 중에는 수정할 수 없습니다. 저장하고 적용한 뒤에 하세요'
               : !d.active_map ? '적용된 지도가 없습니다. 맵 화면에서 지도를 적용하세요'
               : '';
    }
    if((slamOn || !d.active_map) && edit.on) editClose();
    const rm=$('resumeMsg');
    if(rm){ rm.textContent = d.can_resume
              ? "'"+(d.active_map||'')+"' 에 이어서 그립니다"
              : (d.resume_hint||'');
            rm.style.color = d.can_resume ? 'var(--ok)' : 'var(--dim)'; }
    const ttl=$('mapTitle');
    if(ttl) ttl.textContent = slamOn ? (d.paused?'지도 작성 중 (일시정지)':'지도 작성 중')
                            : d.localizing ? '메인 맵 (위치추정 중)' : '매핑 (대기)';

    if(trailMap !== (d.active_map||null)){ clearTrail(); trailMap = d.active_map||null; }
    const am=$('activeMap');
    if(am) am.textContent = d.active_map ? "적용된 지도: '"+d.active_map+"'"
                                         : (slamOn ? '새로 작성 중 (적용된 지도 없음)' : '');
    const se=d.session||{};
    if($('tRec')) $('tRec').innerHTML=
      row('기록', !se.bag_enabled ? '<span class="dim-txt">꺼짐 (record_bag:=false)</span>'
          : se.recording ? '<span class="ok-txt">● 녹화 중</span>'
          : '<span class="dim-txt">대기</span>')+
      row('녹화 크기', fmtMB(se.bag_bytes))+
      row('경과 시간', fmtDur(se.elapsed_sec))+
      row('스캔 수', se.scans==null?'-':se.scans.toLocaleString())+
      row('이동 거리', se.dist_m==null?'-':se.dist_m.toFixed(2)+' m')+
      row('회전량', se.rot_deg==null?'-':se.rot_deg.toFixed(0)+'°');
    $('dot-mapping').className='dot '+(d.slam_running?'on':'off');
    if(!d.slam_running && $('mapMsg') && !$('mapMsg').textContent)
      { $('mapMsg').textContent=d.hint||''; $('mapMsg').style.color='var(--warn)'; }
  }catch(e){}
}

async function loadNodes(){ const d=await api('/nodes');
  $('pNodes').textContent=d.nodes.join('\n'); }
async function loadTopics(){ const d=await api('/topics');
  $('tTopics').innerHTML='<tr><th>토픽</th><th>타입</th></tr>'+
    Object.entries(d.topics).map(([k,v])=>row(k,v.split('/').pop())).join(''); }
let selMap=null, slamOn=false;
function pickMap(n){ selMap=n; renderMapDetail(); }

async function loadMaps(){
  const d=await api('/maps/');
  window._maps=d;
  if(!d.length){
    $('tMaps').innerHTML='<tr><td style="color:var(--dim)">저장된 지도가 없습니다.'+
      ' 매핑 화면에서 만들어 저장하세요.</td></tr>';
    selMap=null; renderMapDetail(); return;
  }
  if(!selMap || !d.some(m=>m.name===selMap)) selMap=d[0].name;
  $('tMaps').innerHTML='<tr><th>이름</th><th>상태</th><th>면적</th><th>용량</th><th>생성</th></tr>'+
    d.map(m=>{
      const sel=m.name===selMap;
      const st = m.active ? '<span class="pill ok">적용중</span>'
               : sel      ? '<span class="pill">선택됨</span>' : '';
      return '<tr onclick="pickMap(\''+m.name+'\')" class="maprow'+
        (sel?' sel':'')+'">'+
        '<td>'+(m.active?'\u25cf ':sel?'\u25b8 ':'\u00a0\u00a0')+m.name+'</td>'+
        '<td>'+st+'</td>'+
        '<td>'+(m.area||'-')+'</td><td>'+(m.size_kb||0)+' KB</td>'+
        '<td>'+new Date(m.create_time*1000).toLocaleString('ko-KR',
          {month:'2-digit',day:'2-digit',hour:'2-digit',minute:'2-digit'})+
        '</td></tr>';
    }).join('');
  renderMapDetail();
}

function renderMapDetail(){
  const m=(window._maps||[]).find(x=>x.name===selMap);
  if($('prevName')) $('prevName').textContent = m ? '- '+m.name : '';
  if(!m){
    if($('mapsPrev')){ $('mapsPrev').removeAttribute('src');
                       $('mapsPrev').style.display='none'; }
    if($('prevEmpty')) $('prevEmpty').style.display='';
    if($('tMapDetail')) $('tMapDetail').innerHTML=
      '<tr><td style="color:var(--dim)">지도를 선택하세요</td></tr>';
    const b0=$('btnApply');
    if(b0){ b0.textContent='이 지도 적용'; b0.disabled=true; }
    if($('mapNote')) $('mapNote').value='';
    return;
  }
  if($('mapsPrev')){ $('mapsPrev').src='/maps/'+m.name+'/preview.png?'+m.create_time;
                     $('mapsPrev').style.display=''; }
  if($('prevEmpty')) $('prevEmpty').style.display='none';
  const ba=$('btnApply');
  if(ba){ ba.textContent = m.active ? '적용중' : '이 지도 적용';
          ba.disabled = !!m.active || !m.has_posegraph; }
  if($('tMapDetail')) $('tMapDetail').innerHTML=
    row('상태', m.active ? '<span class="ok-txt">현재 적용중</span>' : '선택됨')+
    row('픽셀',(m.width||'-')+' x '+(m.height||'-'))+
    row('해상도', m.resolution?m.resolution.toFixed(3)+' m/px':'-')+
    row('실제 면적', m.area||'-')+
    row('이어서 매핑', m.has_posegraph
        ?'<span class="ok-txt">가능</span>'
        :'<span class="warn-txt">불가 (포즈그래프 없음)</span>')+
    row('주행 기록', m.has_bag
        ?'<span class="ok-txt">있음</span> ('+m.bag_mb+' MB)'
        :'<span class="dim-txt">없음</span>')+
    row('매핑 시간', fmtDur(m.duration_sec))+
    row('이동 / 회전',
        (m.distance_m==null?'-':m.distance_m.toFixed(2)+' m')+' / '+
        (m.rotation_deg==null?'-':m.rotation_deg.toFixed(0)+'°'))+
    row('스캔 수', m.scans==null?'-':m.scans.toLocaleString())+
    row('전체 용량', ((m.size_kb||0)/1024).toFixed(1)+' MB');
  if($('mapNote')) $('mapNote').value = m.note||'';
}

async function mapsAct(url){
  const el=$('mapsMsg'); if(el){ el.textContent='처리 중...'; el.style.color='var(--dim)'; }
  try{
    const r=await api(url,{method:'POST'});
    if(el){ el.textContent=r.message||''; el.style.color=r.ok?'var(--ok)':'var(--bad)'; }
  }catch(e){ if(el){ el.textContent='요청 실패'; el.style.color='var(--bad)'; } }
  loadMaps();
}
function mapLoad(){ if(selMap) mapsAct('/maps/'+selMap+'/load'); }
async function mapNote(){
  if(!selMap) return;
  const el=$('mapsMsg'), v=$('mapNote')?$('mapNote').value:'';
  try{
    const r=await api('/maps/'+selMap+'/note',
                      {method:'POST',body:JSON.stringify({note:v})});
    if(el){ el.textContent=r.message||''; el.style.color=r.ok?'var(--ok)':'var(--bad)'; }
  }catch(e){ if(el){ el.textContent='메모 저장 실패'; el.style.color='var(--bad)'; } }
  loadMaps();
}
function mapDelete(){
  if(!selMap) return;
  if(!confirm("'"+selMap+"' 를 삭제합니다. 되돌릴 수 없습니다.")) return;
  const n=selMap; selMap=null; mapsAct('/maps/'+n+'/delete');
}

// ---------------------------------------------------------------- 주기 갱신
async function refresh(){
  try{
    const [s,p,b]=await Promise.all([api('/chassis/status'),api('/chassis/pose'),
                                     api('/battery-state')]);
    status=s; pose=p; batt=b; estop=!!s.emergency_stop_pressed; mode=s.control_mode;
    chip('cLink','연결됨','ok');
    chip('cMode', mode==='manual'?'수동':'자동', mode==='manual'?'warn':'');
    if($('mapMAuto')){
      $('mapMAuto').className=mode==='auto'?'sel':'';
      $('mapMMan').className=mode==='manual'?'sel':'';
    }
    if($('mAuto')){
      $('mAuto').className=mode==='auto'?'sel':'';
      $('mMan').className=mode==='manual'?'sel':'';
      const jb=$('joyBox'); if(jb) jb.className='joybox'+(mode==='manual'?' on':'');
      if(mode!=='manual'&&joy.timer) stopDrive();
      drawJoy();
    }
    const be=$('btnEstop');
    be.textContent=estop?'● 정지 해제':'■ 비상정지';
    be.className='estop'+(estop?' armed':'');

    const ms=s.motors||{}, ks=Object.keys(ms).sort();
    const bad=ks.filter(k=>+ms[k].level>=2);
    chip('cMotor', ks.length?(bad.length?'모터 이상 '+bad.length:'모터 정상'):'모터 없음',
         ks.length?(bad.length?'bad':'ok'):'bad');
    $('dot-motor').className='dot '+(ks.length?(bad.length?'bad':'on'):'off');

    const mkRow=k=>{const m=ms[k],lv=+m.level;
      const c=lv>=2?'bad-txt':lv==1?'warn-txt':'ok-txt';
      return '<tr><td>'+k+'</td><td>'+(m.cmd_rpm||0)+'</td><td>'+(m.act_rpm||0)+
        '</td><td>'+(m.position||0)+'</td><td>'+(m.voltage||0)+' V</td>'+
        '<td class="'+c+'">'+(m.message||'-')+'</td></tr>'; };
    const head='<tr><th>축</th><th>지령</th><th>실측</th><th>위치</th><th>전압</th><th>상태</th></tr>';
    const empty='<tr><td colspan="6" style="color:var(--dim)">base_node 미실행</td></tr>';
    if($('tMotor'))  $('tMotor').innerHTML  = head+(ks.length?ks.map(mkRow).join(''):empty);
    if($('tMotorS')) $('tMotorS').innerHTML = ks.length
      ? ks.map(k=>row('축 '+k.slice(-1), ms[k].act_rpm+' rpm')).join('')
      : row('상태','<span class="bad-txt">base_node 미실행</span>');

    renderBattery(b);
    if($('liftState')){
      const L={up:'올라가는 중 (주행 멈춤)',down:'내려가는 중 (주행 멈춤)',
               top:'올림 완료 — 렉 든 상태, 느리게 주행',bottom:'내림 완료',
               stopped:'중간 정지 — 렉 든 상태로 보고 느리게 주행',unknown:'알 수 없음 (아직 동작 없음)'};
      $('liftState').textContent = s.lift ? (L[s.lift]||s.lift) : '상태 없음 (base_node)';
    }

    // 맵 좌표(TF map→base_footprint)가 있으면 그걸로 로봇을 그린다
    mapPose = pose.map || null;
    if(mapPose){
      const last=trail[trail.length-1];
      if(!last || Math.hypot(mapPose.x-last[0], mapPose.y-last[1])>0.03){
        trail.push([mapPose.x,mapPose.y]);
        if(trail.length>3000) trail.shift();
      }
      if(mapTg('tgFollow') && mapTg('tgFollow').checked){
        view.cx=mapPose.x; view.cy=mapPose.y;
      }
    }
    const poseHtml = pose.error
      ? row('상태','<span class="bad-txt">'+pose.error+'</span>')
      : (mapPose ? row('맵 X','<b>'+mapPose.x.toFixed(3)+' m</b>')+
                   row('맵 Y','<b>'+mapPose.y.toFixed(3)+' m</b>')+
                   row('맵 방위','<b>'+mapPose.yaw_deg.toFixed(1)+'°</b>') : '')+
        row('odom X',pose.x.toFixed(3)+' m')+row('odom Y',pose.y.toFixed(3)+' m')+
        row('방위',pose.yaw_deg.toFixed(1)+'°')+
        row('속도',pose.linear_velocity.toFixed(3)+' m/s')+
        row('각속도',pose.angular_velocity.toFixed(3)+' rad/s');
    if($('tPose')) $('tPose').innerHTML=poseHtml;
  }catch(e){ chip('cLink','서버 연결 끊김','bad'); stopDrive(); }
}

async function refreshScan(){
  if(cur!=='dash'&&cur!=='lidar'&&cur!=='mapping') return;
  try{
    scan=await api('/scan');
    $('dot-lidar').className='dot '+(scan.count?'on':'off');
    if($('tScan')&&!scan.error) $('tScan').innerHTML=
      row('프레임',scan.frame)+row('원본 점수',scan.count_raw)+
      row('유효 점수',scan.count)+
      row('각도 범위',scan.angle_min_deg.toFixed(1)+'° ~ '+scan.angle_max_deg.toFixed(1)+'°');
    draw();
  }catch(e){}
}
async function refreshCam(){
  if(cur!=='camera') return;
  try{ const d=await api('/camera/info');
    $('dot-camera').className='dot '+(d.available?'on':'off');
    $('tCam').innerHTML=row('상태',d.available?'수신 중':'없음')+
      row('해상도',(d.width||'-')+' x '+(d.height||'-'))+row('인코딩',d.encoding||'-');
  }catch(e){}
}

// ---------------------------------------------------------------- POI
// 이름 붙인 지점. 대차 POI 가 가리키는 것은 '로봇이 서는 자리' 가 아니라
// '대차 자리' 다. 로봇이 얼마나 떨어져 설지는 접근거리로 따로 정한다.
// 그래서 대차를 옮겨도 POI 만 다시 찍으면 되고, 거리는 그대로 남는다.
let poiList=[], poiJob=null;
// 등록된 대차 규격. 지도에 렉을 실제 크기로 그리는 데 쓴다.
let cartSpec={};
async function cartSpecLoad(){
  try{
    const d=await api('/carts/specs');
    for(const c of (d.carts||[])) cartSpec[c.name]={w:c.width, d:c.depth};
    const sel=$('poiCart');
    if(sel && !sel.options.length){
      sel.innerHTML = Object.keys(cartSpec)
        .map(n=>'<option value="'+esc(n)+'">'+esc(n)+'</option>').join('')
        + '<option value="">종류 지정 안 함</option>';
    }
  }catch(e){}
}

function poiKindChange(){ poiRefresh(); }

async function poiRefresh(){
  try{
    const d=await api('/pois');
    poiList = d.pois||[];
    mvRenderPts();
    const kind = ($('poiKind')||{}).value || 'cart';
    const see = $('poiSee');
    if(see){
      if(!d.editable){ see.textContent='지도를 적용해야 등록할 수 있습니다.'; }
      else if(kind==='charger'){
        see.innerHTML='<b>충전 스테이션 앞면이 라이다에 보이게</b>(0.5~2.5m) 세우고 등록하세요. '
          +'앞면 V 홈으로 스테이션 자리를 재서, 로봇 뒤끝이 붙은 도킹 자세를 저장합니다. '
          +'[이동]은 스테이션 앞 대기 자리(0.8m)로 갑니다.';
      }
      else if(kind!=='cart'){ see.textContent='로봇이 지금 서 있는 자리를 등록합니다.'; }
      else {
        see.innerHTML='<b>렉 중심에 로봇을 세우고</b> 등록하세요. 도킹을 마쳤을 때의 '
          +'자세로 저장됩니다. 접근 자리는 로봇이 바라보는 쪽으로 '
          +'접근거리만큼 나간 곳으로 자동 계산됩니다. 인식은 필요 없습니다.';
      }
      for(const id of ['poiSoRow','poiCartRow']){
        const e=$(id); if(e) e.style.display = (kind==='cart')?'':'none';
      }
    }
    const t=$('tPoi'); if(!t) return;
    if(!poiList.length){ t.innerHTML=''; return; }
    t.innerHTML = poiList.map((q,i)=>{
      const kd = q.kind==='cart' ? '대차' : q.kind==='charger' ? '충전소' : '지점';
      const so = q.kind==='cart' ? (' · 접근 '+q.standoff.toFixed(1)+'m')
               : q.kind==='charger' ? (' · 대기 '+q.standoff.toFixed(1)+'m')
               : (q.narrow ? ' · 좁은 곳(곧게 진입)' : '');
      return '<tr><td>'+esc(q.name)+'<br><span class="note">'+kd
        +' ('+q.x.toFixed(2)+', '+q.y.toFixed(2)+')'+so+'</span></td>'
        +'<td style="white-space:nowrap">'
        + (q.kind==='cart'
            ? '<button class="rb" onclick="poiJobStart('+i+')">작업</button> '
            : '<button class="rb" onclick="poiGo('+i+')">이동</button> ')
        + (q.kind==='point'
            ? '<button class="rb" onclick="poiNarrow('+i+')">'+(q.narrow?'좁은 곳 끄기':'좁은 곳')+'</button> '
            : '')
        + '<button class="rb" onclick="poiDel('+i+')">삭제</button>'
        +'</td></tr>';
    }).join('');
  }catch(e){}
}
// 결과는 두 군데에 다 쓴다. 도킹 창에서 누른 것이 매핑 화면에만 찍혀
// 창 안에서는 아무 일도 없는 것처럼 보였다(2026-09-16).
function poiSay(m, cls){
  for(const id of ['poiMsg','dockJob']){
    const e=$(id); if(!e) continue;
    if(id==='dockJob' && !dockOn) continue;
    e.textContent=m||''; e.style.color = cls==='bad' ? 'var(--bad)' : '';
  }
}
async function poiSave(){
  const nm=($('poiName')||{}).value||'';
  if(!nm.trim()){ poiSay('이름을 넣어 주세요','bad'); return; }
  const kind=($('poiKind')||{}).value||'cart';
  const body={name:nm.trim(), kind:kind};
  if(kind==='cart'){
    body.standoff = +($('poiSo')||{}).value || 1.8;
    const c=($('poiCart')||{}).value; if(c) body.cart=c;
  }
  try{
    const r=await api('/pois',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    poiSay(r.message, r.ok?'':'bad');
    if(r.ok){ $('poiName').value=''; poiRefresh(); drawMap(); }
  }catch(e){ poiSay('등록하지 못했습니다','bad'); }
}
// 이름 대신 번호로 받는다. 이름을 onclick 문자열에 끼워 넣으면 따옴표가
// 들어간 이름에서 깨진다.
function poiAt(i){ return (poiList[i]||{}).name; }
async function poiDel(i){
  const nm=poiAt(i); if(!nm) return;
  if(!confirm("'"+nm+"' 을(를) 지울까요?")) return;
  try{ const r=await api('/pois/delete',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({name:nm})});
    poiSay(r.message, r.ok?'':'bad'); poiRefresh(); drawMap(); }catch(e){}
}
// 2026-11-16 좁은 곳: 켜면 POI 앞 1m 축 위 자리까지 간 뒤 축을 따라 곧게 들어가고,
// 안에서 돌 자리가 없으면 곧게 후진해 나온 뒤 출발한다.
async function poiNarrow(i){
  const q=poiList[i]; if(!q) return;
  const body=Object.assign({}, q, {narrow: !q.narrow});
  try{ const r=await api('/pois',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});
    poiSay(r.ok ? ("'"+q.name+"' 좁은 곳 "+(body.narrow?'켬 (곧게 진입)':'끔')) : r.message, r.ok?'':'bad');
    poiRefresh(); }catch(e){}
}
async function poiGo(i){
  const nm=poiAt(i); if(!nm) return;
  try{ const r=await api('/pois/goto',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({name:nm})});
    poiSay(r.message, r.ok?'':'bad'); }catch(e){}
}
async function poiJobStart(i){
  const nm=poiAt(i); if(!nm) return false;
  try{ const r=await api('/job/start',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({poi:nm})});
    poiSay(r.message, r.ok?'':'bad'); return !!r.ok; }catch(e){ return false; }
}
async function poiJobCancel(){
  try{ const r=await api('/job/cancel',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({})});
    poiSay(r.message); }catch(e){}
}
// 작업 진행 상태. 어디까지 왔는지 한 줄로 보여 준다.
const JOB_TXT={idle:'', moving:'이동 중', looking:'대차 찾는 중',
  staging:'돌아서는 중', entering:'후진 진입 중', preflip:'캐스터 정렬 중', exiting:'빠져나오는 중', exit_fix:'방향 맞추는 중',
  done:'완료', failed:'실패', canceled:'취소됨'};
async function poiJobPoll(){
  try{
    poiJob = await api('/job/status');
    const h=$('poiHint');
    const st=poiJob.state,
          busy=['moving','looking','staging','entering','preflip','exiting','exit_fix'].indexOf(st)>=0;
    let txt = JOB_TXT[st]||st;
    if(busy){
      txt = '<b>'+txt+'</b> — '+esc(poiJob.message||'');
      if(poiJob.step) txt += ' · '+esc(poiJob.step);
      txt += ' <button class="rb" onclick="poiJobCancel()">중단</button>';
    } else if(st==='failed'){
      txt = '<span style="color:var(--bad)">실패 — '+esc(poiJob.message||'')+'</span>';
    } else if(st==='done'){
      txt = '<span style="color:var(--ok)">'+esc(poiJob.message||'완료')+'</span>';
    } else { txt=''; }
    if(h) h.innerHTML = txt;
    // 지도 위 띠 — 창을 닫아도 여기서 보인다
    for(const [bar,msg] of [['jobBar','jbMsg'],['jobBar2','jbMsg2']]){
      const b=$(bar); if(!b) continue;
      b.classList.toggle('on', busy);
      const m=$(msg);
      if(m && busy) m.textContent =
        (JOB_TXT[st]||st) + (poiJob.step ? ' · '+poiJob.step : '');
    }
    if(dockOn){
      const d=$('dockJob'); if(d) d.innerHTML = txt || '렉 또는 충전소를 고르세요';
      const sb=$('dockStop');
      if(sb) sb.style.display = busy ? '' : 'none';
      dockDraw();
    }
  }catch(e){}
}

// ---------------------------------------------------------------- 도킹 창
// 대시보드에서 렉을 골라 진입시킨다. 목록은 POI 에서 그대로 온다 —
// 등록은 매핑 화면에서 하고, 여기서는 고르기만 한다.
let dockOn=false;
let dockNav=null;   // nav2 가 받을 준비가 됐나
function dockOpen(){ dockOn=true; $('dockBg').classList.add('on');
  poiRefresh().then(dockDraw); dockDraw();
  // nav2 가 안 떠 있으면 눌러도 시작되지 않는다. 누르기 전에 알려 준다.
  api('/nav/status').then(d=>{ dockNav=!!(d&&d.available); dockDraw(); })
                    .catch(()=>{ dockNav=false; dockDraw(); }); }
function dockClose(){ dockOn=false; $('dockBg').classList.remove('on'); }
function dockDraw(){
  if(!dockOn) return;
  const el=$('dockList'); if(!el) return;
  const carts = poiList.filter(q=>q.kind==='cart');
  const chgs = poiList.filter(q=>q.kind==='charger');
  if(!carts.length && !chgs.length){
    el.innerHTML='<p class="note" style="margin:0">등록된 렉 · 충전소가 없습니다. '
      +'매핑 화면에서 POI 로 등록하세요.</p>';
    return;
  }
  const busy = poiJob && ['moving','looking','staging','entering','preflip','exiting','exit_fix']
                          .indexOf(poiJob.state)>=0;
  // 자동 모드가 아니면 작업이 시작되지 않는다. 누르기 전에 알려 준다.
  const blocked = (mode!=='auto') || estop || dockNav===false;
  const why = estop ? '비상정지 상태입니다.'
            : (mode!=='auto' ? '수동 모드입니다. 자동으로 바꿔야 시작됩니다.'
            : (dockNav===false ? '자율주행(nav2)이 준비되지 않았습니다.' : ''));
  el.innerHTML = carts.map(q=>{
    const i = poiList.indexOf(q);
    const on = busy && poiJob.poi===q.name;
    return '<div class="row" style="justify-content:space-between;'
      +'align-items:center;gap:10px;padding:9px 2px;border-top:1px solid var(--line)">'
      +'<div><b>'+esc(q.name)+'</b><br><span class="note">'
      + (q.cart? esc(q.cart)+' · ':'')
      +'('+q.x.toFixed(2)+', '+q.y.toFixed(2)+') · '
      +q.standoff.toFixed(1)+'m 앞에서 인식</span></div>'
      +'<button class="rb'+(on?' sel':'')+'" '+((busy||blocked)?'disabled':'')
      +' onclick="dockGo('+i+')">'+(on?'진행 중':'렉 진입')+'</button></div>';
  }).join('')
  // 충전소: 대기 자리(앞 standoff) -> V 홈으로 정렬 -> 뒤로 들어가 접촉에서 선다
  // (charger_dock.py, 2026-11-12). 충전 전류 확인은 나중에.
  + chgs.map(q=>{
    const i = poiList.indexOf(q);
    const on = cdock && cdock.busy && cdock.poi===q.name;
    return '<div class="row" style="justify-content:space-between;'
      +'align-items:center;gap:10px;padding:9px 2px;border-top:1px solid var(--line)">'
      +'<div><b>'+esc(q.name)+'</b> <span class="note">충전소</span><br><span class="note">'
      +q.standoff.toFixed(1)+'m 앞에서 인식 · 뒤로 들어가 접촉에서 정지</span></div>'
      +'<button class="rb'+(on?' sel':'')+'" '+((busy||blocked||(cdock&&cdock.busy))?'disabled':'')
      +' onclick="dockCharger('+i+')">'+(on?'진행 중':'충전소 도킹')+'</button></div>';
  }).join('');
  if(why) el.innerHTML += '<p class="note" style="color:var(--bad);margin:10px 0 0">'
    + why + (mode!=='auto' && !estop && dockNav!==false
             ? ' <button class="rb" onclick="setMode(\'auto\')">자동으로</button>'
             : '') + '</p>';
}
async function dockCharger(i){
  const el=$('dockJob');
  if(el){ el.textContent='시작하는 중…'; el.style.color=''; }
  const nm=poiAt(i); if(!nm) return;
  try{ const r=await api('/charger/dock',{method:'POST',
      headers:{'Content-Type':'application/json'},body:JSON.stringify({name:nm})});
    if(el){ el.textContent=r.message||''; el.style.color=r.ok?'':'var(--bad)'; }
    if(r.ok) dockClose();
  }catch(e){ if(el){ el.textContent='보내지 못했습니다'; el.style.color='var(--bad)'; } }
}
async function cdockCancel(){
  try{ await api('/charger/cancel',{method:'POST',
      headers:{'Content-Type':'application/json'},body:'{}'}); mvPoll(); }catch(e){}
}
async function dockGo(i){
  // ⚠ 예전에는 먼저 닫고, 실패하면 다시 열었다. 그런데 poiSay 는 팝업이
  //   열려 있을 때만 써 넣으므로 실패 이유가 통째로 사라지고, 팝업만 다시
  //   뜨는 것으로 보였다 — nav2 가 죽어 있을 때 무한 반복이었다
  //   (2026-09-21 지적). 성공했을 때만 닫는다.
  const el=$('dockJob');
  if(el){ el.textContent='시작하는 중…'; el.style.color=''; }
  const ok = await poiJobStart(i);
  if(ok){ dockClose(); await poiJobPoll(); return; }
  dockDraw();          // 이유는 poiSay 가 dockJob 에 써 놓았다
}

// ---------------------------------------------------------------- 주행 속도
// 자율주행(nav2)의 목표 속도. 끄는 동안이 아니라 손을 뗄 때 한 번만 보낸다.
// 끄는 내내 보내면 nav2 파라미터 서비스에 지령이 쏟아진다.
// 서버가 값을 파일에 남기므로 새로고침해도, nav2 를 껐다 켜도 유지된다.
let navSpd=null;
function navSpeedShow(v){ const t=$('navSpdV'); if(t) t.textContent=(+v).toFixed(2); }
async function navSpeedSet(v){
  navSpd=+v;
  try{
    const r=await api('/nav/speed',{method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({speed:+v})});
    if(r.speed!=null){ navSpd=r.speed; navSpeedShow(r.speed); }
    const m=$('navSpdMsg'); if(m) m.textContent=r.message||'';
  }catch(e){ const m=$('navSpdMsg'); if(m) m.textContent='속도를 바꾸지 못했습니다'; }
}
// 서버가 들고 있는 값으로 슬라이더를 맞춘다. 사람이 잡고 있을 때는 건드리지 않는다.
function navSpeedSync(v){
  if(v==null) return;
  navSpd=v;
  const el=$('navSpd');
  if(el && document.activeElement!==el) el.value=v.toFixed(2);
  navSpeedShow(v);
}

['vLin','vAng'].forEach(id=>{ const el=$(id); if(!el) return;
  el.oninput=()=>$(id+'V').textContent=(+el.value).toFixed(2); });

buildNav();
loadUI();
cartSpecLoad();
go(location.hash.slice(1) || 'dash', true);
// 토글을 만질 때마다 저장한다
document.addEventListener('change', e=>{
  if(e.target && UI_TOGGLES.indexOf(e.target.id)>=0){ saveUI(); drawMap(); }
});
// 로봇 추적으로 화면이 따라간 위치도 이따금 저장해 둔다
setInterval(saveUI, 5000);
addEventListener('hashchange',()=>go(location.hash.slice(1)||'dash', true));
document.addEventListener('input',e=>{ if(e.target.id==='rng') draw();
  if(e.target.id==='vLin'||e.target.id==='vAng')
    $(e.target.id+'V').textContent=(+e.target.value).toFixed(2); });
setInterval(refresh,500);
setInterval(refreshScan,200);
// POI 는 자주 바뀌지 않지만 작업 진행은 자주 봐야 한다
setInterval(poiRefresh, 2000);
setInterval(poiJobPoll, 500);

// 렉 인식 결과 — 토글이 켜져 있거나 카메라 화면일 때만 받아온다
async function refreshCarts(){
  const on = cartOn;
  // 대시보드에서 켜면 렉 인식 패널(결과 표)을 띄운다. 카메라 두 대는 지도 밑에 늘 보인다.
  const pan=$('cartPanel');
  if(pan) pan.style.display = (cur==='dash' && on) ? '' : 'none';
  if(cur==='dash') startCamD();
  // 2026-11-13 대시보드는 렉 인식 결과를 정면 카메라에 늘 겹쳐 그린다
  if(!(on || cur==='camera' || cur==='dash')){ carts=null; cartsErr=null; return; }
  // 받아오기와 그리기를 분리한다. 한 try 로 묶으면 그리기에서 난 오류가
  // 받아온 자료까지 지워 버려, 원인을 찾기 어려워진다.
  try{ carts = await api('/carts'); cartsErr=null; }
  catch(e){ carts=null; cartsErr=String(e && e.message || e).slice(0,60); }
  try{
    if(cur==='camera') drawCamOverlay('imgCam','cvCam','camCartMsg');
    if(cur==='dash') drawCamOverlay('imgCamD','cvCamD','cartMsgD','tCartD');
  }catch(e){ cartsErr='그리기 오류: '+String(e && e.message || e).slice(0,50); }
}
function startCamD(){ camSrc('imgCamD','/camera/color.mjpg'); camSrc('imgCamDn','/camera/down.mjpg'); }
setInterval(refreshCarts,400);

// 카메라 영상 위에 인식된 다리를 겹쳐 그린다.
// 카메라가 차체 앞끝에 있어 가까운 다리는 화각 밖이라 안 보일 수 있다.
function x0(p,kx){ return p.u*kx; }
function drawCamOverlay(imgId, cvId, msgId, tblId){
  imgId=imgId||'imgCam'; cvId=cvId||'cvCam'; msgId=msgId||'camCartMsg';
  const cv=$(cvId), im=$(imgId);
  if(!cv||!im||!im.clientWidth) return;
  cv.width=im.clientWidth; cv.height=im.clientHeight;
  const g=cv.getContext('2d');
  g.clearRect(0,0,cv.width,cv.height);
  const msg=$(msgId), tbl=tblId?$(tblId):null;
  if(!carts||!carts.carts||!carts.carts.length){
    if(msg){ msg.textContent='렉이 인식되지 않았습니다'; msg.style.color='var(--dim)'; }
    if(tbl) tbl.innerHTML=row('상태','<span class="dim-txt">인식 없음</span>')+
      row('다리 후보',(carts&&carts.candidates!=null?carts.candidates:'-')+' 개');
    return;
  }
  // 여러 개가 잡히면 '대상' 으로 뽑힌 것만 카메라 화면에 그린다.
  const list = carts.carts.filter(c=>c.target!==false);
  for(const c of (list.length?list:carts.carts)){
    const sz=c.cam_size, uv=c.cam_uv;
    if(!sz||!uv) continue;
    const kx=cv.width/sz[0], ky=cv.height/sz[1];
    let seen=0;
    let hidden=0;
    for(const p of uv){
      if(!p) continue;
      // 초록 = 기둥 확인 / 주황 = 그 자리에 기둥이 없음
      // 회색 점선 = 앞 다리에 가려 카메라가 못 봄 (판정 안 함)
      // 가려진 다리를 주황으로 그리면 '엉뚱한 데 박스가 갔다' 로 보인다.
      if(p.occl){
        hidden++;
        g.strokeStyle='#8f97a8'; g.lineWidth=1.5; g.setLineDash([4,3]);
        g.strokeRect(x0(p,kx)-9, p.v*ky-4, 18, Math.max(26,(p.h||0)*ky+8));
        g.setLineDash([]);
        g.fillStyle='#8f97a8'; g.font='12px system-ui';
        g.fillText(p.why||'가려짐', x0(p,kx)+12, p.v*ky+10);
        continue;
      }
      seen++;
      const x=p.u*kx, y=p.v*ky, h=(p.h||40)*ky;
      const col=p.ok?'#4de0a0':'#ff8f5c';
      g.strokeStyle=col; g.lineWidth=2;
      g.strokeRect(x-9,y-4,18,h);
      g.fillStyle=col;
      g.beginPath(); g.arc(x,y,5,0,7); g.fill();
      g.font='12px system-ui';
      g.fillText(p.z.toFixed(2)+'m'+(p.ratio!=null?' · '+Math.round(p.ratio*100)+'%':''), x+12, y+10);
    }
    // st 는 아래 표에서도 쓴다. if(msg) 블록 안에 const 로 두었더니
    // 표를 그릴 때 ReferenceError 가 나서, 대시보드 렉 패널의 표가
    // 계속 비어 있었다(try/catch 가 삼켜 눈에 안 띄었다).
    const st=c.cam_ok===true?'통과':c.cam_ok===false?'실패':'보류';
    if(msg){
      msg.textContent=c.cart+' 인식 — 카메라 검증 '+st+' ('+c.cam_legs+
                      '), 확인한 다리 '+seen+'개'+
                      (hidden?' · 가려진 다리 '+hidden+'개':'');
      msg.style.color=c.cam_ok===false?'var(--bad)':'var(--ok)';
    }
    if(tbl) tbl.innerHTML=
      row('종류','<b>'+c.cart+'</b>')+
      row('일치도',(c.score*100).toFixed(0)+' %')+
      row('카메라 검증',st+' ('+c.cam_legs+')')+
      row('로봇 기준','앞 '+c.x.toFixed(3)+' m, 옆 '+c.y.toFixed(3)+' m')+
      row('방향',c.yaw_deg.toFixed(1)+'°')+
      row('실측 크기',(c.a*1000).toFixed(0)+' x '+(c.b*1000).toFixed(0)+' mm')+
      row('규격 오차',c.err_mm.toFixed(0)+' mm')+
      row('다리 후보',(carts.candidates||0)+' 개 중 4개 선택')+
      // 도킹 어긋남 — 대차 중심과 로봇 구동축 중심의 차이.
      // 진입 제어는 이 세 값을 0 으로 몰아가면 된다.
      (carts.align && carts.align.ready
        ? row('도킹 어긋남',
            '앞뒤 <b>'+carts.align.forward.toFixed(3)+'</b> m · 좌우 <b>'+
            carts.align.lateral.toFixed(3)+'</b> m · 회전 <b>'+
            carts.align.yaw_deg.toFixed(1)+'</b>°')
        : row('도킹 어긋남','<span class="dim-txt">'+
            ((carts.align&&carts.align.reason)||'-')+'</span>'))+
      (carts.carts.length>1
        ? row('다른 후보','<span class="warn-txt">'+(carts.carts.length-1)+
              ' 개</span> <span class="dim-txt">(규격만 맞고 카메라 확인 안 됨)</span>')
        : '');
  }
}
setInterval(refreshCam,1000);
setInterval(refreshMap,1500);
setInterval(()=>{ if(cur==='mapping') loadMapping(); },2000);
setInterval(()=>{ if(cur==='log') loadLog(); },1500);
refresh();
</script></body></html>
"""
