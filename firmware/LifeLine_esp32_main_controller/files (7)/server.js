const express = require('express');
const bodyParser = require('body-parser');
const cors = require('cors');
const app = express();
app.use(cors());
app.use(bodyParser.json());
const PORT = 3000;

let users = {}, blockedRooms = [], nodeDanger = {};
let exitLoad = { Node_Exit_Main: 0, Node_Exit_TopRight: 0 };

// ── Graph (matches Unity export exactly) ──
const graph = {
    Node_Passage:             ["Door_Passage_Hall","Door_Passage_Female","Door_Passage_Male","Door_Kitchen_Passage"],
    Node_OpenOffice_Top:      ["Door_Lounge_OpenOfficeTop","Door_OpenOfficeTop_Meeting","Door_OpenOffice_Connection_Right","Door_OpenOffice_Connection_Left"],
    Node_OpenOffice_Bottom:   ["Door_Hall_OpenOffice","Door_OpenOfficeBottom_Meeting","Door_OpenOfficeBottom_Private","Door_OpenOffice_Connection_Right","Door_OpenOffice_Connection_Left"],
    Node_Meeting_TopRight:    ["Door_OpenOfficeTop_Meeting","Door_MeetingTop_Exit"],
    Node_Exit_Main:           ["Door_Hall_Exit_Main"],
    Node_Exit_TopRight:       ["Door_MeetingTop_Exit"],
    Node_FemaleBathroom:      ["Door_Passage_Female"],
    Node_Hall:                ["Door_Passage_Hall","Door_Hall_OpenOffice","Door_Hall_Private","Door_Hall_Exit_Main"],
    Node_Kitchen:             ["Door_Lounge_Kitchen","Door_Kitchen_Passage"],
    Node_PrivateOffice:       ["Door_Hall_Private","Door_OpenOfficeBottom_Private"],
    Node_Lounge:              ["Door_Lounge_Kitchen","Door_Lounge_OpenOfficeTop"],
    Node_MaleBathroom:        ["Door_Passage_Male"],
    Node_Meeting_BottomRight: ["Door_OpenOfficeBottom_Meeting"],
    Door_Lounge_Kitchen:              ["Node_Lounge","Node_Kitchen"],
    Door_Lounge_OpenOfficeTop:        ["Node_Lounge","Node_OpenOffice_Top"],
    Door_Kitchen_Passage:             ["Node_Kitchen","Node_Passage"],
    Door_Passage_Hall:                ["Node_Passage","Node_Hall"],
    Door_Passage_Female:              ["Node_Passage","Node_FemaleBathroom"],
    Door_Passage_Male:                ["Node_Passage","Node_MaleBathroom"],
    Door_Hall_OpenOffice:             ["Node_Hall","Node_OpenOffice_Bottom"],
    Door_Hall_Private:                ["Node_Hall","Node_PrivateOffice"],
    Door_Hall_Exit_Main:              ["Node_Hall","Node_Exit_Main"],
    Door_OpenOfficeTop_Meeting:       ["Node_OpenOffice_Top","Node_Meeting_TopRight"],
    Door_OpenOfficeBottom_Meeting:    ["Node_OpenOffice_Bottom","Node_Meeting_BottomRight"],
    Door_OpenOfficeBottom_Private:    ["Node_PrivateOffice","Node_OpenOffice_Bottom"],
    Door_OpenOffice_Connection_Right: ["Node_OpenOffice_Top","Node_OpenOffice_Bottom"],
    Door_OpenOffice_Connection_Left:  ["Node_OpenOffice_Top","Node_OpenOffice_Bottom"],
    Door_MeetingTop_Exit:             ["Node_Meeting_TopRight","Node_Exit_TopRight"],
};

// ── Unity world positions (for A* heuristic) ──
const positions = {
    Node_Passage:{x:1.537,y:0,z:2.049},
    Node_OpenOffice_Top:{x:-0.275,y:0,z:0.019},
    Node_OpenOffice_Bottom:{x:-0.195,y:0,z:1.329},
    Node_Meeting_TopRight:{x:-2.808,y:0,z:-0.271},
    Node_Exit_Main:{x:0.675,y:0,z:3.009},
    Node_Exit_TopRight:{x:-2.861,y:0,z:-1.071},
    Node_FemaleBathroom:{x:2.121,y:0,z:2.730},
    Node_Hall:{x:0.785,y:0,z:2.349},
    Node_Kitchen:{x:1.595,y:0,z:1.309},
    Node_PrivateOffice:{x:-0.615,y:0,z:2.139},
    Node_Lounge:{x:1.535,y:0,z:0.449},
    Node_MaleBathroom:{x:2.121,y:0,z:1.926},
    Node_Meeting_BottomRight:{x:-2.285,y:0,z:1.479},
    Door_Lounge_Kitchen:{x:1.504,y:0,z:0.712},
    Door_Lounge_OpenOfficeTop:{x:1.332,y:0,z:0.440},
    Door_Kitchen_Passage:{x:1.504,y:0,z:1.644},
    Door_Passage_Hall:{x:1.326,y:0,z:2.099},
    Door_Passage_Female:{x:1.820,y:0,z:2.688},
    Door_Passage_Male:{x:1.809,y:0,z:1.812},
    Door_Hall_OpenOffice:{x:0.710,y:0,z:1.740},
    Door_Hall_Private:{x:-0.360,y:0,z:2.749},
    Door_Hall_Exit_Main:{x:0.719,y:0,z:3.005},
    Door_OpenOfficeTop_Meeting:{x:-1.702,y:0,z:0.015},
    Door_OpenOfficeBottom_Meeting:{x:-1.702,y:0,z:1.470},
    Door_OpenOfficeBottom_Private:{x:-0.584,y:0,z:1.650},
    Door_OpenOffice_Connection_Right:{x:-1.515,y:0,z:0.739},
    Door_OpenOffice_Connection_Left:{x:1.135,y:0,z:0.739},
    Door_MeetingTop_Exit:{x:-2.827,y:0,z:-1.269},
};

const DANGER_W=2.0, SWITCH_T=1.5, CONG_W=2.0;

function h(a,b){
    const pa=positions[a],pb=positions[b];
    if(!pa||!pb)return 999;
    return Math.sqrt((pa.x-pb.x)**2+(pa.z-pb.z)**2);
}

function astar(start,goal,xtraCost=0){
    if(!graph[start])return[];
    let open=[start],came={},g={},f={};
    for(let n in graph){g[n]=Infinity;f[n]=Infinity;}
    g[start]=0; f[start]=h(start,goal);
    while(open.length){
        open.sort((a,b)=>f[a]-f[b]);
        let cur=open.shift();
        if(cur===goal)return reconstruct(came,cur);
        for(let nb of(graph[cur]||[])){
            if(blockedRooms.includes(nb))continue;
            let cost=g[cur]+h(cur,nb)+(nodeDanger[nb]||0)*DANGER_W+(nb===goal?xtraCost:0);
            if(cost<g[nb]){came[nb]=cur;g[nb]=cost;f[nb]=cost+h(nb,goal);if(!open.includes(nb))open.push(nb);}
        }
    }
    return[];
}

function reconstruct(came,cur){
    let p=[cur];
    while(came[cur]){cur=came[cur];p.unshift(cur);}
    return p;
}

function bestExit(start,currentExit){
    let best=null,bestScore=Infinity,bestPath=[];
    for(let exit of['Node_Exit_Main','Node_Exit_TopRight']){
        if(blockedRooms.includes(exit))continue;
        const congCost=(exitLoad[exit]||0)*CONG_W;
        const path=astar(start,exit,congCost);
        if(!path.length)continue;
        const score=pathDist(path)+congCost - (exit===currentExit?SWITCH_T:0);
        if(score<bestScore){bestScore=score;best=exit;bestPath=path;}
    }
    return{path:bestPath,exit:best};
}

function pathDist(p){
    let d=0;
    for(let i=1;i<p.length;i++)d+=h(p[i-1],p[i]);
    return d;
}

function recalcLoad(){
    exitLoad={Node_Exit_Main:0,Node_Exit_TopRight:0};
    for(let u in users){const e=users[u].assignedExit;if(e&&exitLoad[e]!==undefined)exitLoad[e]++;}
}

function density(){
    const d={};
    for(let u in users){const r=users[u].room;if(r)d[r]=(d[r]||0)+1;}
    return d;
}

function cleanup(){
    const now=Date.now();
    for(let u in users){if(now-users[u].lastSeen>10000)delete users[u];}
    recalcLoad();
}

// ── API ──
app.post('/updatePosition',(req,res)=>{
    const{userId,room}=req.body;
    if(!userId||!room)return res.status(400).send("Missing");
    if(!users[userId])users[userId]={room:null,assignedExit:null,lastSeen:0};
    users[userId].room=room;
    users[userId].lastSeen=Date.now();
    res.sendStatus(200);
});

app.get('/path/:uid',(req,res)=>{
    const u=users[req.params.uid];
    if(!u||!u.room)return res.json({path:[],exit:null,exitLoad});
    const{path,exit}=bestExit(u.room,u.assignedExit);
    if(!path.length)return res.json({path:[],exit:null,exitLoad});
    if(u.assignedExit!==exit){u.assignedExit=exit;recalcLoad();}
    res.json({path,exit,exitLoad});
});

app.post('/blocked',(req,res)=>{
    blockedRooms=req.body.rooms||[];
    for(let u in users){if(blockedRooms.includes(users[u].assignedExit))users[u].assignedExit=null;}
    recalcLoad();
    res.sendStatus(200);
});

app.post('/danger',(req,res)=>{
    nodeDanger=req.body.danger||{};
    res.sendStatus(200);
});

app.get('/status',(req,res)=>{
    cleanup();
    res.json({
        userCount:Object.keys(users).length,
        users:Object.entries(users).map(([id,u])=>({
            id:id.slice(-6),room:u.room,assignedExit:u.assignedExit,
            lastSeen:Math.round((Date.now()-u.lastSeen)/1000)+'s ago'
        })),
        exitLoad,roomDensity:density(),blockedRooms,nodeDanger,
        dangerActive:Object.values(nodeDanger).some(v=>v>0)
    });
});

// ── Dashboard ──
app.get('/dashboard',(req,res)=>{
res.send(`<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>Evac Control</title>
<style>
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
html,body{height:100%;overflow-x:hidden;background:#0a0a0a;color:#ddd;font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',sans-serif;}
body{display:flex;flex-direction:column;}

/* Header */
.hdr{flex-shrink:0;padding:10px 16px;background:#111;border-bottom:1px solid #1e1e1e;display:flex;justify-content:space-between;align-items:center;}
.hdr h1{color:#ff6b35;font-size:clamp(14px,4vw,18px);font-weight:800;}
.live{display:flex;align-items:center;gap:5px;font-size:12px;color:#4caf50;font-weight:600;}
.dot{width:7px;height:7px;border-radius:50%;background:#4caf50;animation:pulse 1.4s infinite;}
@keyframes pulse{0%,100%{opacity:1}50%{opacity:.15}}

/* Tabs */
.tabs{flex-shrink:0;display:flex;background:#0d0d0d;border-bottom:1px solid #1a1a1a;}
.tab{flex:1;padding:11px 4px;text-align:center;font-size:clamp(11px,3vw,13px);color:#444;cursor:pointer;border-bottom:2px solid transparent;transition:.2s;font-weight:700;}
.tab.active{color:#ff6b35;border-bottom-color:#ff6b35;}

/* Pages */
.pages{flex:1;overflow-y:auto;}
.page{display:none;padding:12px;}
.page.active{display:block;}

/* Map */
.map-wrap{width:100%;background:#0c0c0c;border-radius:12px;border:1px solid #1e1e1e;overflow:hidden;margin-bottom:10px;}
.map-wrap svg{width:100%;height:auto;display:block;}

/* Room states */
.room{cursor:pointer;transition:fill .25s,filter .25s;}
.room:hover{filter:brightness(1.25);}
.room.selected{stroke:#ff6b35!important;stroke-width:2.5!important;}
.room.fire-src{fill:#5a0000!important;animation:flk 0.9s infinite alternate;}
@keyframes flk{from{filter:drop-shadow(0 0 4px #ff110055)}to{filter:drop-shadow(0 0 12px #ff3300bb)}}
.room.fire-hi{fill:#7b2211!important;}
.room.fire-med{fill:#7d5a08!important;}
.room.fire-lo{fill:#4a4a08!important;}
.room.crowded{stroke:#2196f3!important;stroke-width:2!important;}

/* Info card */
.info{background:#111;border-radius:12px;padding:12px 14px;border:1px solid #1e1e1e;margin-bottom:10px;}
.info-lbl{font-size:10px;color:#444;text-transform:uppercase;letter-spacing:1px;margin-bottom:5px;font-weight:700;}
.sel-name{font-size:clamp(16px,5vw,22px);font-weight:800;color:#ff6b35;}
.sel-exit{font-size:12px;color:#888;margin-top:4px;}
.sel-path{font-size:10px;color:#444;margin-top:5px;line-height:1.8;word-break:break-word;}

/* Buttons */
.btn{width:100%;padding:13px;border-radius:12px;border:none;font-size:clamp(13px,3.5vw,15px);font-weight:700;cursor:pointer;margin-bottom:8px;letter-spacing:.2px;}
.btn:active{opacity:.75;transform:scale(.97);}
.btn-pos{background:#ff6b35;color:#fff;}
.btn-fire{background:#7b0000;color:#fff;}
.btn-clear{background:#0d3520;color:#4caf50;}
.btn:disabled{background:#161616;color:#333;cursor:default;transform:none;}

/* Stats */
.card{background:#111;border-radius:12px;padding:12px 14px;border:1px solid #1e1e1e;margin-bottom:10px;}
.card-ttl{font-size:10px;color:#444;text-transform:uppercase;letter-spacing:1px;margin-bottom:10px;font-weight:700;}
.bignum{font-size:clamp(36px,12vw,52px);font-weight:800;color:#ff6b35;letter-spacing:-2px;}
.erow{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid #161616;}
.erow:last-child{border-bottom:none;}
.ebar{height:5px;border-radius:3px;background:#1a1a1a;margin-top:4px;width:clamp(80px,28vw,140px);}
.ebar-fill{height:100%;border-radius:3px;transition:width .5s;}
.ecnt{font-size:clamp(20px,6vw,28px);font-weight:800;}
.ecnt.g{color:#4caf50;}.ecnt.y{color:#ff9800;}.ecnt.r{color:#f44336;}
.drow,.urow{display:flex;justify-content:space-between;padding:6px 0;border-bottom:1px solid #141414;font-size:clamp(11px,3vw,13px);}
.drow:last-child,.urow:last-child{border-bottom:none;}
.badge{display:inline-flex;align-items:center;padding:3px 10px;border-radius:20px;font-size:11px;font-weight:700;margin-right:5px;margin-bottom:5px;}
.bfire{background:#6b0000;color:#fff;}.bsafe{background:#0d3520;color:#4caf50;}
.sbrow{display:flex;gap:6px;flex-wrap:wrap;margin-bottom:10px;}
.legend{display:flex;flex-wrap:wrap;gap:8px 14px;padding:6px 2px;font-size:11px;color:#555;margin-top:2px;}
@keyframes arPulse{from{transform:scale(1);opacity:.85}to{transform:scale(1.12);opacity:1}}
</style>
</head>
<body>

<div class="hdr">
  <h1>🚨 Evac Control</h1>
  <div class="live"><div class="dot"></div>LIVE</div>
</div>

<div class="tabs">
  <div class="tab active" onclick="showTab('map')">🗺️ Floor Plan</div>
  <div class="tab" onclick="showTab('stats')">📊 Stats</div>
  <div class="tab" onclick="showTab('ar')">📷 AR View</div>
</div>

<div class="pages">

<!-- MAP PAGE -->
<div class="page active" id="page-map">

  <div class="info">
    <div class="info-lbl">Selected Room</div>
    <div class="sel-name" id="sel-room">Tap a room on the map</div>
    <div class="sel-exit" id="sel-exit"></div>
    <div class="sel-path" id="sel-path"></div>
  </div>

  <div class="map-wrap">
  <svg viewBox="0 0 100 100" xmlns="http://www.w3.org/2000/svg" preserveAspectRatio="xMidYMid meet">

    <!-- Dark background -->
    <rect width="100" height="100" fill="#0c0c0c"/>

    <!--
    ╔══════════════════════════════════════════════════════╗
    ║  FLOOR PLAN — SVG coordinate system                 ║
    ║                                                      ║
    ║  Derived from Unity X/Z node positions:             ║
    ║    X: -3.2 → +2.5  maps to  SVG x: 0 → 100         ║
    ║    Z: -1.5 → +3.3  maps to  SVG y: 0 → 100         ║
    ║                                                      ║
    ║  Column layout (left → right):                       ║
    ║  x=0-18:   Exit TR | Mtg Top R | Mtg Bot R          ║
    ║  x=18-68:  Open Office Top | Open Office Bottom     ║
    ║  x=42-68:  Hall (overlaps OO bottom) | Exit Main    ║
    ║  x=18-42:  Private Office (bottom)                  ║
    ║  x=68-93:  Lounge | Kitchen | Passage               ║
    ║  x=93-100: Female WC | Male WC (beside Passage)     ║
    ╚══════════════════════════════════════════════════════╝
    -->

    <!-- ── EXIT TOP RIGHT ── -->
    <rect id="room-Node_Exit_TopRight" class="room"
          x="0" y="0" width="18" height="12"
          fill="#0a2010" stroke="#1a5a1a" stroke-width=".5"
          onclick="selectRoom('Node_Exit_TopRight')"/>
    <text x="9" y="5" text-anchor="middle" fill="#3ab83a" font-size="2.4" font-weight="800" pointer-events="none">EXIT</text>
    <text x="9" y="9.5" text-anchor="middle" fill="#3ab83a" font-size="2" pointer-events="none">Top Right</text>

    <!-- ── MEETING TOP RIGHT ── -->
    <rect id="room-Node_Meeting_TopRight" class="room"
          x="0" y="12" width="18" height="28"
          fill="#0c1520" stroke="#1a2535" stroke-width=".5"
          onclick="selectRoom('Node_Meeting_TopRight')"/>
    <text x="9" y="24" text-anchor="middle" fill="#6a9abf" font-size="2.6" font-weight="700" pointer-events="none">Mtg</text>
    <text x="9" y="30" text-anchor="middle" fill="#6a9abf" font-size="2.6" font-weight="700" pointer-events="none">Top R</text>

    <!-- ── MEETING BOTTOM RIGHT ── -->
    <rect id="room-Node_Meeting_BottomRight" class="room"
          x="0" y="40" width="18" height="28"
          fill="#0c1520" stroke="#1a2535" stroke-width=".5"
          onclick="selectRoom('Node_Meeting_BottomRight')"/>
    <text x="9" y="52" text-anchor="middle" fill="#6a9abf" font-size="2.6" font-weight="700" pointer-events="none">Mtg</text>
    <text x="9" y="58" text-anchor="middle" fill="#6a9abf" font-size="2.6" font-weight="700" pointer-events="none">Bot R</text>

    <!-- ── OPEN OFFICE TOP ── -->
    <rect id="room-Node_OpenOffice_Top" class="room"
          x="18" y="12" width="50" height="28"
          fill="#0d0d1e" stroke="#15153a" stroke-width=".5"
          onclick="selectRoom('Node_OpenOffice_Top')"/>
    <text x="43" y="28" text-anchor="middle" fill="#7777cc" font-size="3.5" font-weight="700" pointer-events="none">Open Office Top</text>

    <!-- ── OPEN OFFICE BOTTOM ── -->
    <rect id="room-Node_OpenOffice_Bottom" class="room"
          x="18" y="40" width="50" height="28"
          fill="#0d0d1e" stroke="#15153a" stroke-width=".5"
          onclick="selectRoom('Node_OpenOffice_Bottom')"/>
    <text x="43" y="56" text-anchor="middle" fill="#7777cc" font-size="3.2" font-weight="700" pointer-events="none">Open Office Bottom</text>

    <!-- ── LOUNGE — top of right column ── -->
    <rect id="room-Node_Lounge" class="room"
          x="68" y="12" width="25" height="28"
          fill="#1a0d0d" stroke="#2d1515" stroke-width=".5"
          onclick="selectRoom('Node_Lounge')"/>
    <text x="80.5" y="28" text-anchor="middle" fill="#cc7777" font-size="2.8" font-weight="700" pointer-events="none">Lounge</text>

    <!-- ── KITCHEN — mid of right column ── -->
    <rect id="room-Node_Kitchen" class="room"
          x="68" y="40" width="25" height="28"
          fill="#0d1a0d" stroke="#152515" stroke-width=".5"
          onclick="selectRoom('Node_Kitchen')"/>
    <text x="80.5" y="56" text-anchor="middle" fill="#77cc77" font-size="2.8" font-weight="700" pointer-events="none">Kitchen</text>

    <!-- ── PRIVATE OFFICE ── -->
    <rect id="room-Node_PrivateOffice" class="room"
          x="18" y="68" width="24" height="20"
          fill="#120d1e" stroke="#1e1530" stroke-width=".5"
          onclick="selectRoom('Node_PrivateOffice')"/>
    <text x="30" y="79" text-anchor="middle" fill="#aa77cc" font-size="2.4" font-weight="700" pointer-events="none">Private</text>
    <text x="30" y="84" text-anchor="middle" fill="#aa77cc" font-size="2.4" font-weight="700" pointer-events="none">Office</text>

    <!-- ── HALL ── -->
    <rect id="room-Node_Hall" class="room"
          x="42" y="68" width="26" height="20"
          fill="#141414" stroke="#252525" stroke-width=".5"
          onclick="selectRoom('Node_Hall')"/>
    <text x="55" y="80" text-anchor="middle" fill="#aaa" font-size="2.8" font-weight="700" pointer-events="none">Hall</text>

    <!-- ── PASSAGE — bottom of right column ── -->
    <rect id="room-Node_Passage" class="room"
          x="68" y="68" width="25" height="20"
          fill="#1a1a1a" stroke="#2a2a2a" stroke-width=".5"
          onclick="selectRoom('Node_Passage')"/>
    <text x="80.5" y="80" text-anchor="middle" fill="#aaa" font-size="2.8" font-weight="700" pointer-events="none">Passage</text>

    <!-- ── FEMALE WC — far right, top, beside Passage ── -->
    <rect id="room-Node_FemaleBathroom" class="room"
          x="93" y="68" width="7" height="10"
          fill="#0d0d1c" stroke="#15152a" stroke-width=".5"
          onclick="selectRoom('Node_FemaleBathroom')"/>
    <text x="96.5" y="73.5" text-anchor="middle" fill="#7777bb" font-size="2" font-weight="700" pointer-events="none">F</text>
    <text x="96.5" y="77" text-anchor="middle" fill="#7777bb" font-size="1.8" pointer-events="none">WC</text>

    <!-- ── MALE WC — far right, bottom, beside Passage ── -->
    <rect id="room-Node_MaleBathroom" class="room"
          x="93" y="78" width="7" height="10"
          fill="#0d0d1c" stroke="#15152a" stroke-width=".5"
          onclick="selectRoom('Node_MaleBathroom')"/>
    <text x="96.5" y="83.5" text-anchor="middle" fill="#7777bb" font-size="2" font-weight="700" pointer-events="none">M</text>
    <text x="96.5" y="87" text-anchor="middle" fill="#7777bb" font-size="1.8" pointer-events="none">WC</text>

    <!-- ── EXIT MAIN — below Hall ── -->
    <rect id="room-Node_Exit_Main" class="room"
          x="42" y="88" width="26" height="10"
          fill="#0a2010" stroke="#1a5a1a" stroke-width=".7"
          onclick="selectRoom('Node_Exit_Main')"/>
    <text x="55" y="94.5" text-anchor="middle" fill="#3ab83a" font-size="2.4" font-weight="800" pointer-events="none">EXIT MAIN</text>

    <!--
    ╔═════════════════════════════╗
    ║  WALLS                      ║
    ╚═════════════════════════════╝
    All walls drawn once, no duplicates.
    -->

    <!-- Outer building border -->
    <rect x="0" y="0" width="100" height="98" fill="none" stroke="#3a3a3a" stroke-width="1"/>

    <!-- Left outer wall (meeting col, y=12 to y=68) -->
    <line x1="0" y1="12" x2="0" y2="68" stroke="#3a3a3a" stroke-width="1"/>

    <!-- Exit TR bottom -->
    <line x1="0" y1="12" x2="18" y2="12" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- OO Top top wall -->
    <line x1="18" y1="12" x2="68" y2="12" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Right col top wall (Lounge) -->
    <line x1="68" y1="12" x2="93" y2="12" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- WC top wall -->
    <line x1="93" y1="12" x2="100" y2="12" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Meeting Top | Bottom divider -->
    <line x1="0" y1="40" x2="18" y2="40" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Meeting col right wall -->
    <line x1="18" y1="12" x2="18" y2="68" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- OO Top | Bottom divider -->
    <line x1="18" y1="40" x2="68" y2="40" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Lounge | Kitchen divider -->
    <line x1="68" y1="40" x2="93" y2="40" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Right col left wall (full height) -->
    <line x1="68" y1="12" x2="68" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- WC left wall (beside Passage) -->
    <line x1="93" y1="68" x2="93" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Female | Male WC divider -->
    <line x1="93" y1="78" x2="100" y2="78" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Meeting col bottom / bottom row top -->
    <line x1="0" y1="68" x2="18" y2="68" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Bottom row top (centre) -->
    <line x1="18" y1="68" x2="68" y2="68" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Kitchen | Passage divider -->
    <line x1="68" y1="68" x2="93" y2="68" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- WC top wall (flush with Kitchen|Passage) -->
    <line x1="93" y1="68" x2="100" y2="68" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Private | Hall divider -->
    <line x1="42" y1="68" x2="42" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Hall | Passage divider (bottom row) -->
    <line x1="68" y1="68" x2="68" y2="98" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Private Office bottom -->
    <line x1="18" y1="88" x2="42" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Hall bottom / Exit Main top -->
    <line x1="42" y1="88" x2="68" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Passage bottom -->
    <line x1="68" y1="88" x2="93" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- WC bottom -->
    <line x1="93" y1="88" x2="100" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!-- Private Office left wall -->
    <line x1="18" y1="68" x2="18" y2="88" stroke="#3a3a3a" stroke-width=".8"/>

    <!--
    ╔═════════════════════════════╗
    ║  DOOR GAPS                  ║
    ║  Each door = gap in wall    ║
    ║  Drawn over wall in bg clr  ║
    ╚═════════════════════════════╝

    Doors derived from graph connections:
    Door_MeetingTop_Exit        → y=12 top of Mtg TR
    Door_OpenOfficeTop_Meeting  → x=18 between Mtg TR and OO Top
    Door_OpenOfficeBottom_Meeting → x=18 between Mtg BR and OO Bottom
    Door_OpenOffice_Connection_Left  → y=40 left gap OO Top↔Bottom
    Door_OpenOffice_Connection_Right → y=40 right gap OO Top↔Bottom
    Door_Lounge_OpenOfficeTop   → x=68 between OO Top and Lounge
    Door_Lounge_Kitchen         → y=40 between Lounge and Kitchen
    Door_Kitchen_Passage        → y=68 between Kitchen and Passage
    Door_Passage_Hall           → x=68 between Passage and Hall
    Door_Passage_Female         → x=93 Passage→Female WC
    Door_Passage_Male           → x=93 Passage→Male WC
    Door_Hall_OpenOffice        → y=68 Hall→OO Bottom
    Door_Hall_Private           → x=42 Hall→Private Office
    Door_Hall_Exit_Main         → y=88 Hall→Exit Main
    Door_OpenOfficeBottom_Private → y=68 OO Bottom→Private Office
    -->

    <!-- Exit TR → Mtg TR (top wall gap) -->
    <line x1="6" y1="12" x2="12" y2="12" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Mtg TR → OO Top (x=18 wall gap) -->
    <line x1="18" y1="20" x2="18" y2="27" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Mtg BR → OO Bottom (x=18 wall gap) -->
    <line x1="18" y1="50" x2="18" y2="57" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- OO Top ↔ OO Bottom — left opening (y=40) -->
    <line x1="29" y1="40" x2="37" y2="40" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- OO Top ↔ OO Bottom — right opening (y=40) -->
    <line x1="53" y1="40" x2="61" y2="40" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- OO Top → Lounge (x=68 wall gap) -->
    <line x1="68" y1="19" x2="68" y2="26" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Lounge → Kitchen (y=40 gap in right col) -->
    <line x1="74" y1="40" x2="82" y2="40" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Kitchen → Passage (y=68 gap in right col) -->
    <line x1="74" y1="68" x2="82" y2="68" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Passage → Hall (x=68 left wall of Passage) -->
    <line x1="68" y1="72" x2="68" y2="79" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Passage → Female WC (x=93) -->
    <line x1="93" y1="70" x2="93" y2="75" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Passage → Male WC (x=93) -->
    <line x1="93" y1="80" x2="93" y2="85" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- OO Bottom → Private Office (y=68) -->
    <line x1="23" y1="68" x2="32" y2="68" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Hall → OO Bottom (y=68 top of Hall) -->
    <line x1="51" y1="68" x2="61" y2="68" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Hall → Private Office (x=42) -->
    <line x1="42" y1="73" x2="42" y2="80" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Hall → Exit Main (y=88) -->
    <line x1="50" y1="88" x2="62" y2="88" stroke="#0c0c0c" stroke-width="2.2"/>

    <!-- Route and user layers (drawn by JS) -->
    <g id="route-layer"></g>
    <g id="user-layer"></g>

  </svg>
  </div>

  <button class="btn btn-pos" id="btn-send" onclick="sendPos()" disabled>📍 Set My Position Here</button>
  <button class="btn btn-fire" onclick="triggerFire()">🔥 Trigger Fire at Selected Room</button>
  <button class="btn btn-clear" onclick="clearFire()">✅ Clear All Fire &amp; Danger</button>

  <div class="legend">
    <span>🟠 You &nbsp;🔵 Others</span>
    <span style="color:#5a0000">■ Fire source</span>
    <span style="color:#7d5a08">■ Danger zone</span>
    <span style="color:#2a8a2a">■ Exit</span>
    <span style="color:#ff6b35">--- Evacuation route</span>
  </div>

</div>

<!-- STATS PAGE -->
<div class="page" id="page-stats">
  <div class="sbrow" id="sbadges"></div>
  <div class="card"><div class="card-ttl">Active Users</div><div class="bignum" id="uc">—</div></div>
  <div class="card"><div class="card-ttl">Exit Load</div><div id="el"></div></div>
  <div class="card"><div class="card-ttl">Room Density</div><div id="dr"></div></div>
  <div class="card"><div class="card-ttl">User List</div><div id="ur"></div></div>
</div>

</div><!-- end pages -->

<!-- ═══════════════════════════════════════════════════
     AR CAMERA PAGE — full-screen camera with arrow overlay
═══════════════════════════════════════════════════ -->
<div class="page" id="page-ar" style="padding:0;position:relative;background:#000;min-height:calc(100vh - 100px);">

  <!-- Camera feed -->
  <video id="ar-video" autoplay playsinline muted
    style="width:100%;height:calc(100vh - 100px);object-fit:cover;display:block;background:#000;"></video>

  <!-- Arrow canvas overlaid on camera -->
  <canvas id="ar-canvas"
    style="position:absolute;top:0;left:0;width:100%;height:calc(100vh - 100px);pointer-events:none;"></canvas>

  <!-- Status bar -->
  <div id="ar-status" style="
    position:absolute;top:0;left:0;right:0;
    background:linear-gradient(to bottom,rgba(0,0,0,.7),transparent);
    padding:12px 16px;display:flex;justify-content:space-between;align-items:center;">
    <div style="color:#ff6b35;font-weight:800;font-size:15px;">🚨 AR Evacuation</div>
    <div id="ar-exit-label" style="color:#4caf50;font-weight:700;font-size:13px;">Waiting for position...</div>
  </div>

  <!-- Direction indicator (large central arrow when route exists) -->
  <div id="ar-direction" style="
    position:absolute;bottom:100px;left:0;right:0;
    display:flex;flex-direction:column;align-items:center;
    pointer-events:none;opacity:0;transition:opacity .4s;">
    <div id="ar-arrow-icon" style="
      font-size:72px;line-height:1;
      filter:drop-shadow(0 0 16px #ff6b35cc);
      animation:arPulse 1.2s infinite alternate;"></div>
    <div id="ar-direction-label" style="
      color:#fff;font-size:14px;font-weight:700;
      background:rgba(0,0,0,.55);padding:4px 14px;border-radius:20px;margin-top:6px;"></div>
  </div>

  <!-- Bottom control bar -->
  <div style="
    position:absolute;bottom:0;left:0;right:0;
    background:linear-gradient(to top,rgba(0,0,0,.85),transparent);
    padding:14px 16px 24px;display:flex;gap:10px;align-items:center;">
    <button id="ar-start-btn" onclick="startAR()" style="
      flex:1;padding:13px;border-radius:12px;border:none;
      background:#ff6b35;color:#fff;font-size:14px;font-weight:700;cursor:pointer;">
      📷 Start Camera
    </button>
    <button onclick="stopAR()" style="
      padding:13px 16px;border-radius:12px;border:none;
      background:rgba(255,255,255,.15);color:#fff;font-size:13px;font-weight:700;cursor:pointer;">
      ✕
    </button>
  </div>

  <!-- No fire warning -->
  <div id="ar-no-fire" style="
    position:absolute;top:50%;left:50%;transform:translate(-50%,-50%);
    background:rgba(0,0,0,.8);border-radius:16px;padding:24px 28px;
    text-align:center;display:none;border:1px solid #333;">
    <div style="font-size:36px;margin-bottom:10px;">🟢</div>
    <div style="color:#4caf50;font-weight:700;font-size:16px;margin-bottom:6px;">All Clear</div>
    <div style="color:#888;font-size:13px;">No fire detected.<br>AR arrows appear when fire is triggered.</div>
  </div>

</div>


<script>
// ═══════════════════════════════════════════════════════
//  NODE CENTRES — exact centres of each SVG room rect
//  Formula: cx = rect.x + rect.width/2
//           cy = rect.y + rect.height/2
//
//  Room rects:
//  Exit_TopRight:     x=0,  y=0,  w=18, h=12  → cx=9,    cy=6
//  Meeting_TopRight:  x=0,  y=12, w=18, h=28  → cx=9,    cy=26
//  Meeting_BottomR:   x=0,  y=40, w=18, h=28  → cx=9,    cy=54
//  OpenOffice_Top:    x=18, y=12, w=50, h=28  → cx=43,   cy=26
//  OpenOffice_Bottom: x=18, y=40, w=50, h=28  → cx=43,   cy=54
//  Lounge:            x=68, y=12, w=25, h=28  → cx=80.5, cy=26
//  Kitchen:           x=68, y=40, w=25, h=28  → cx=80.5, cy=54
//  PrivateOffice:     x=18, y=68, w=24, h=20  → cx=30,   cy=78
//  Hall:              x=42, y=68, w=26, h=20  → cx=55,   cy=78
//  Passage:           x=68, y=68, w=25, h=20  → cx=80.5, cy=78
//  FemaleBathroom:    x=93, y=68, w=7,  h=10  → cx=96.5, cy=73
//  MaleBathroom:      x=93, y=78, w=7,  h=10  → cx=96.5, cy=83
//  Exit_Main:         x=42, y=88, w=26, h=10  → cx=55,   cy=93
//
//  Door waypoints = midpoint on the wall where the door is:
// ═══════════════════════════════════════════════════════
const NC = {
    Node_Exit_TopRight:       {x:9,    y:6},
    Node_Meeting_TopRight:    {x:9,    y:26},
    Node_Meeting_BottomRight: {x:9,    y:54},
    Node_OpenOffice_Top:      {x:43,   y:26},
    Node_OpenOffice_Bottom:   {x:43,   y:54},
    Node_Lounge:              {x:80.5, y:26},
    Node_Kitchen:             {x:80.5, y:54},
    Node_PrivateOffice:       {x:30,   y:78},
    Node_Hall:                {x:55,   y:78},
    Node_Passage:             {x:80.5, y:78},
    Node_FemaleBathroom:      {x:96.5, y:73},
    Node_MaleBathroom:        {x:96.5, y:83},
    Node_Exit_Main:           {x:55,   y:93},
    // Door waypoints (midpoint of the door gap on the wall)
    Door_MeetingTop_Exit:             {x:9,    y:12},   // top wall y=12
    Door_OpenOfficeTop_Meeting:       {x:18,   y:23.5}, // x=18, mid of y=20-27
    Door_OpenOfficeBottom_Meeting:    {x:18,   y:53.5}, // x=18, mid of y=50-57
    Door_OpenOffice_Connection_Left:  {x:33,   y:40},   // y=40, mid of x=29-37
    Door_OpenOffice_Connection_Right: {x:57,   y:40},   // y=40, mid of x=53-61
    Door_Lounge_OpenOfficeTop:        {x:68,   y:22.5}, // x=68, mid of y=19-26
    Door_Lounge_Kitchen:              {x:78,   y:40},   // y=40, mid of x=74-82
    Door_Kitchen_Passage:             {x:78,   y:68},   // y=68, mid of x=74-82
    Door_Passage_Hall:                {x:68,   y:75.5}, // x=68, mid of y=72-79
    Door_Passage_Female:              {x:93,   y:72.5}, // x=93, mid of y=70-75
    Door_Passage_Male:                {x:93,   y:82.5}, // x=93, mid of y=80-85
    Door_OpenOfficeBottom_Private:    {x:27.5, y:68},   // y=68, mid of x=23-32
    Door_Hall_OpenOffice:             {x:56,   y:68},   // y=68, mid of x=51-61
    Door_Hall_Private:                {x:42,   y:76.5}, // x=42, mid of y=73-80
    Door_Hall_Exit_Main:              {x:56,   y:88},   // y=88, mid of x=50-62
};

// ── Identity ──
let myId = localStorage.getItem('evac_uid');
if(!myId){myId='user_'+Math.random().toString(36).slice(2,10).toUpperCase();localStorage.setItem('evac_uid',myId);}

let sel=null, autoInt=null, myPath=[];

// ── Tab switching ──
function showTab(t){
    document.querySelectorAll('.tab').forEach((el,i)=>el.classList.toggle('active',
        (t==='map'&&i===0)||(t==='stats'&&i===1)||(t==='ar'&&i===2)));
    document.querySelectorAll('.page').forEach(p=>p.classList.remove('active'));
    document.getElementById('page-'+t).classList.add('active');
    // Stop camera if leaving AR tab
    if(t!=='ar') stopAR();
}

// ── Room selection ──
function selectRoom(name){
    sel=name;
    document.querySelectorAll('.room').forEach(r=>r.classList.remove('selected'));
    const el=document.getElementById('room-'+name);if(el)el.classList.add('selected');
    const label=name.replace('Node_','').replace(/_/g,' ');
    document.getElementById('sel-room').textContent=label;
    document.getElementById('sel-exit').textContent='';
    document.getElementById('sel-path').textContent='';
    document.getElementById('btn-send').disabled=false;
}

// ── Send position ──
async function sendPos(){
    if(!sel)return;
    if(autoInt)clearInterval(autoInt);
    await ping(sel);
    autoInt=setInterval(()=>{if(sel)ping(sel);},1000);
}

async function ping(room){
    try{
        await fetch('/updatePosition',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({userId:myId,room})});
        const d=await(await fetch('/path/'+myId)).json();
        myPath=d.path||[];
        if(d.exit){
            document.getElementById('sel-exit').textContent='→ Exit: '+d.exit.replace('Node_Exit_','').replace(/_/g,' ');
        } else {
            document.getElementById('sel-exit').textContent='⚠ No route — all exits blocked!';
        }
        const steps=myPath.filter(n=>n.startsWith('Node_')).map(n=>n.replace('Node_','').replace(/_/g,' '));
        document.getElementById('sel-path').textContent=steps.length?'Path: '+steps.join(' → '):'';
        drawRoute(myPath);
    }catch(e){}
}

// ── Fire controls ──
async function triggerFire(){
    if(!sel){alert('Select a room first');return;}
    await fetch('/blocked',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({rooms:[sel]})});
    document.getElementById('sel-exit').textContent='🔥 Fire triggered at '+sel.replace('Node_','').replace(/_/g,' ');
}

async function clearFire(){
    await fetch('/blocked',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({rooms:[]})});
    await fetch('/danger',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({danger:{}})});
    document.querySelectorAll('.room').forEach(r=>r.classList.remove('fire-src','fire-hi','fire-med','fire-lo'));
    myPath=[];drawRoute([]);
    document.getElementById('sel-exit').textContent='';
    document.getElementById('sel-path').textContent='';
}

// ── Route drawing ──
function mkEl(tag){return document.createElementNS('http://www.w3.org/2000/svg',tag);}

function drawRoute(path){
    const L=document.getElementById('route-layer');L.innerHTML='';
    if(!path||path.length<2)return;
    const pts=path.map(n=>NC[n]).filter(Boolean);
    if(pts.length<2)return;

    // Glow underline
    const glow=mkEl('polyline');
    glow.setAttribute('points',pts.map(p=>p.x+','+p.y).join(' '));
    glow.setAttribute('fill','none');glow.setAttribute('stroke','#ff6b35');
    glow.setAttribute('stroke-width','2');glow.setAttribute('stroke-opacity','.12');
    glow.setAttribute('stroke-linecap','round');L.appendChild(glow);

    // Dashed route line
    const line=mkEl('polyline');
    line.setAttribute('points',pts.map(p=>p.x+','+p.y).join(' '));
    line.setAttribute('fill','none');line.setAttribute('stroke','#ff6b35');
    line.setAttribute('stroke-width','0.8');line.setAttribute('stroke-dasharray','2.5,1.5');
    line.setAttribute('stroke-linecap','round');L.appendChild(line);

    // Directional arrows at midpoint of each segment
    for(let i=1;i<pts.length;i++){
        const a=pts[i-1],b=pts[i];
        const dx=b.x-a.x,dy=b.y-a.y;
        const dist=Math.sqrt(dx*dx+dy*dy);
        if(dist<2)continue;
        const ang=Math.atan2(dy,dx)*180/Math.PI;
        const mx=(a.x+b.x)/2,my=(a.y+b.y)/2;
        const ar=mkEl('polygon');
        ar.setAttribute('points','-1.5,-0.8 1.5,0 -1.5,0.8');
        ar.setAttribute('fill','#ff6b35');ar.setAttribute('opacity','.85');
        ar.setAttribute('transform','translate('+mx+','+my+') rotate('+ang+')');
        L.appendChild(ar);
    }

    // Destination pulse
    const dp=pts[pts.length-1];
    const pulse=mkEl('circle');pulse.setAttribute('cx',dp.x);pulse.setAttribute('cy',dp.y);
    pulse.setAttribute('r','3');pulse.setAttribute('fill','#4caf50');pulse.setAttribute('opacity','.2');L.appendChild(pulse);
    const dc=mkEl('circle');dc.setAttribute('cx',dp.x);dc.setAttribute('cy',dp.y);
    dc.setAttribute('r','1.8');dc.setAttribute('fill','#4caf50');dc.setAttribute('stroke','white');dc.setAttribute('stroke-width','.35');L.appendChild(dc);
}

// ── Fire/danger visual ──
function applyFire(danger){
    document.querySelectorAll('.room').forEach(r=>r.classList.remove('fire-src','fire-hi','fire-med','fire-lo'));
    Object.entries(danger||{}).forEach(([n,v])=>{
        const el=document.getElementById('room-'+n);if(!el)return;
        if(v>=10)el.classList.add('fire-src');
        else if(v>=7)el.classList.add('fire-hi');
        else if(v>=4)el.classList.add('fire-med');
        else if(v>=1)el.classList.add('fire-lo');
    });
}

// ── Crowd density visual ──
function applyDensity(den){
    document.querySelectorAll('.room').forEach(r=>r.classList.remove('crowded'));
    Object.entries(den||{}).forEach(([r,c])=>{
        if(c>=2){const el=document.getElementById('room-'+r);if(el)el.classList.add('crowded');}
    });
}

// ── User dots ──
function drawUsers(users){
    const L=document.getElementById('user-layer');L.innerHTML='';
    users.forEach((u,i)=>{
        const pos=NC[u.room];if(!pos)return;
        const isMe=u.id===myId.slice(-6);
        const ox=isMe?0:(i%3===0?2.5:i%3===1?-2.5:0);
        const oy=isMe?0:(i%2===0?-2:2);
        const ring=mkEl('circle');ring.setAttribute('cx',pos.x+ox);ring.setAttribute('cy',pos.y+oy);
        ring.setAttribute('r','2.8');ring.setAttribute('fill',isMe?'#ff6b3520':'#2196f320');L.appendChild(ring);
        const c=mkEl('circle');c.setAttribute('cx',pos.x+ox);c.setAttribute('cy',pos.y+oy);
        c.setAttribute('r',isMe?'2':'1.6');c.setAttribute('fill',isMe?'#ff6b35':'#2196f3');
        c.setAttribute('stroke','white');c.setAttribute('stroke-width','.3');L.appendChild(c);
        if(isMe){
            const t=mkEl('text');t.setAttribute('x',pos.x+ox);t.setAttribute('y',pos.y+oy-3.2);
            t.setAttribute('text-anchor','middle');t.setAttribute('font-size','1.8');
            t.setAttribute('fill','#ff6b35');t.setAttribute('font-weight','bold');
            t.textContent='YOU';L.appendChild(t);
        }
    });
}

// ── Stats panel ──
function updateStats(d){
    document.getElementById('uc').textContent=d.userCount;
    document.getElementById('sbadges').innerHTML=
        (d.dangerActive?'<span class="badge bfire">🔥 FIRE ACTIVE</span>':'<span class="badge bsafe">✅ All Clear</span>')+
        (d.blockedRooms.length?'<span class="badge bfire">⛔ '+d.blockedRooms.length+' blocked</span>':'');
    const mx=Math.max(...Object.values(d.exitLoad),1);
    document.getElementById('el').innerHTML=Object.entries(d.exitLoad).map(([e,c])=>{
        const lbl=e.replace('Node_Exit_','').replace(/_/g,' ');
        const pct=Math.round((c/mx)*100);
        const col=c===0?'#4caf50':c<=2?'#ff9800':'#f44336';
        const cls=c===0?'g':c<=2?'y':'r';
        return '<div class="erow"><div><div style="font-size:13px;color:#ccc">'+lbl+'</div>'+
               '<div class="ebar"><div class="ebar-fill" style="width:'+pct+'%;background:'+col+'"></div></div></div>'+
               '<div class="ecnt '+cls+'">'+c+'</div></div>';
    }).join('');
    const de=Object.entries(d.roomDensity||{}).sort((a,b)=>b[1]-a[1]);
    document.getElementById('dr').innerHTML=de.length===0
        ?'<div style="color:#333;font-size:12px;padding:4px 0">No users tracked</div>'
        :de.map(([r,c])=>'<div class="drow"><span style="color:#aaa">'+r.replace('Node_','').replace(/_/g,' ')+'</span>'+
            '<span style="color:'+(c>=3?'#f44336':c>=2?'#ff9800':'#4caf50')+';font-weight:800">'+c+'</span></div>').join('');
    document.getElementById('ur').innerHTML=d.users.length===0
        ?'<div style="color:#333;font-size:12px;padding:4px 0">No active users</div>'
        :d.users.map(u=>'<div class="urow">'+
            '<div><div style="font-size:10px;color:#444;font-family:monospace">'+u.id+(u.id===myId.slice(-6)?' 👤':'')+'</div>'+
            '<div style="color:#ccc">'+(u.room?u.room.replace('Node_','').replace(/_/g,' '):'—')+'</div></div>'+
            '<div style="text-align:right">'+
            '<div style="color:#ff6b35;font-size:11px">'+(u.assignedExit?'→ '+u.assignedExit.replace('Node_Exit_','').replace(/_/g,' '):'—')+'</div>'+
            '<div style="color:#333;font-size:10px">'+u.lastSeen+'</div></div></div>').join('');
}

// ── Main poll loop ──
async function poll(){
    try{
        const d=await(await fetch('/status')).json();
        applyFire(d.nodeDanger);
        applyDensity(d.roomDensity);
        drawUsers(d.users);
        updateStats(d);
        if(myPath.length)drawRoute(myPath);
    }catch(e){}
}

// ═══════════════════════════════════════════════════════
//  AR CAMERA VIEW
//  - Opens device back camera
//  - Reads current user path from server
//  - Computes direction to NEXT node in path
//  - Draws animated directional arrow on canvas overlay
// ═══════════════════════════════════════════════════════

let arStream = null, arAnimFrame = null, arRunning = false;

async function startAR(){
    if(arRunning) return;
    const video = document.getElementById('ar-video');
    const btn   = document.getElementById('ar-start-btn');
    try{
        // Request back camera (environment)
        arStream = await navigator.mediaDevices.getUserMedia({
            video:{ facingMode:{ideal:'environment'}, width:{ideal:1280}, height:{ideal:720} },
            audio:false
        });
        video.srcObject = arStream;
        await video.play();
        arRunning = true;
        btn.textContent = '📷 Camera On';
        btn.style.background = '#1a4731';
        resizeARCanvas();
        window.addEventListener('resize', resizeARCanvas);
        arLoop();
    } catch(e){
        alert('Camera access denied or unavailable.\\nPlease allow camera permission and try again.');
        console.error('AR camera error:', e);
    }
}

function stopAR(){
    arRunning = false;
    if(arStream){ arStream.getTracks().forEach(t=>t.stop()); arStream=null; }
    if(arAnimFrame){ cancelAnimationFrame(arAnimFrame); arAnimFrame=null; }
    const video = document.getElementById('ar-video');
    video.srcObject = null;
    const btn = document.getElementById('ar-start-btn');
    btn.textContent = '📷 Start Camera';
    btn.style.background = '#ff6b35';
    clearARCanvas();
    document.getElementById('ar-direction').style.opacity = '0';
    document.getElementById('ar-no-fire').style.display = 'none';
    document.getElementById('ar-exit-label').textContent = 'Waiting for position...';
}

function resizeARCanvas(){
    const canvas = document.getElementById('ar-canvas');
    const video  = document.getElementById('ar-video');
    canvas.width  = video.clientWidth;
    canvas.height = video.clientHeight;
}

function clearARCanvas(){
    const canvas = document.getElementById('ar-canvas');
    const ctx = canvas.getContext('2d');
    ctx.clearRect(0,0,canvas.width,canvas.height);
}

// ── Direction logic ──
// Maps consecutive path nodes to compass/screen direction
// based on their NC (SVG) positions. Since SVG is top-down:
//   NC x increases → RIGHT
//   NC y increases → DOWN (further into building)
// We compare current room NC to next room NC to get heading.

function getDirectionArrow(fromNode, toNode){
    const from = NC[fromNode], to = NC[toNode];
    if(!from || !to) return null;
    const dx = to.x - from.x;
    const dy = to.y - from.y;  // SVG y: down = positive
    const dist = Math.sqrt(dx*dx + dy*dy);
    if(dist < 1) return null;
    const angle = Math.atan2(dy, dx) * 180 / Math.PI; // degrees, 0=right, 90=down
    // Map angle to emoji arrow + text
    const dirs = [
        { min:-22.5,  max:22.5,  arrow:'➡️', label:'Turn RIGHT' },
        { min:22.5,   max:67.5,  arrow:'↘️', label:'Go FORWARD-RIGHT' },
        { min:67.5,   max:112.5, arrow:'⬇️', label:'Go STRAIGHT' },
        { min:112.5,  max:157.5, arrow:'↙️', label:'Go FORWARD-LEFT' },
        { min:157.5,  max:180,   arrow:'⬅️', label:'Turn LEFT' },
        { min:-180,   max:-157.5,arrow:'⬅️', label:'Turn LEFT' },
        { min:-157.5, max:-112.5,arrow:'↖️', label:'Go BACK-LEFT' },
        { min:-112.5, max:-67.5, arrow:'⬆️', label:'Go BACK' },
        { min:-67.5,  max:-22.5, arrow:'↗️', label:'Go BACK-RIGHT' },
    ];
    for(const d of dirs){
        if(angle >= d.min && angle < d.max) return d;
    }
    return { arrow:'⬇️', label:'Go STRAIGHT' };
}

let arPathCache = [];
let arFireActive = false;
let arPollTimer = null;

// Poll server for current path every 1.5s during AR
async function arPollServer(){
    if(!arRunning) return;
    try{
        const status = await (await fetch('/status')).json();
        arFireActive = status.dangerActive;
        // Find my user in the list
        const me = status.users.find(u => u.id === myId.slice(-6));
        if(me && me.room){
            const pathResp = await (await fetch('/path/'+myId)).json();
            arPathCache = pathResp.path || [];
            const exit = pathResp.exit;
            if(exit){
                document.getElementById('ar-exit-label').textContent =
                    '→ Exit: ' + exit.replace('Node_Exit_','').replace(/_/g,' ');
            } else {
                document.getElementById('ar-exit-label').textContent = '⚠ No route available';
                arPathCache = [];
            }
        }
    } catch(e){}
}

// ── AR render loop ──
function arLoop(){
    if(!arRunning){ return; }
    renderAR();
    arAnimFrame = requestAnimationFrame(arLoop);
}

let arLastPoll = 0;
function renderAR(){
    const now = Date.now();
    // Poll server every 1500ms
    if(now - arLastPoll > 1500){
        arLastPoll = now;
        arPollServer();
    }

    const canvas = document.getElementById('ar-canvas');
    const ctx    = canvas.getContext('2d');
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0,0,W,H);

    const noFire   = document.getElementById('ar-no-fire');
    const dirPanel = document.getElementById('ar-direction');

    // ── No fire → show "all clear" overlay ──
    if(!arFireActive || !arPathCache || arPathCache.length < 2){
        dirPanel.style.opacity = '0';
        if(arPathCache && arPathCache.length < 2 && arFireActive){
            // Fire but no path — trapped
            noFire.style.display = 'none';
            drawARText(ctx, W/2, H/2, '⛔ No safe route', '#f44336', 22);
            drawARText(ctx, W/2, H/2+36, 'Find nearest exit manually', '#fff', 14);
        } else {
            noFire.style.display = arRunning ? 'flex' : 'none';
            noFire.style.flexDirection = 'column';
            noFire.style.alignItems = 'center';
        }
        return;
    }
    noFire.style.display = 'none';

    // ── We have a fire + a path — show directional arrows ──
    // Find current room (first node in path is user's room)
    const currentRoom = arPathCache[0];
    const nextNode    = arPathCache[1];  // next step
    const finalExit   = arPathCache[arPathCache.length - 1];

    const dir = getDirectionArrow(currentRoom, nextNode);

    if(dir){
        // Update big emoji arrow overlay
        document.getElementById('ar-arrow-icon').textContent    = dir.arrow;
        document.getElementById('ar-direction-label').textContent = dir.label;
        dirPanel.style.opacity = '1';

        // Draw canvas arrows on camera feed
        drawCanvasArrows(ctx, W, H, dir);

        // Draw path progress dots at bottom
        drawPathProgress(ctx, W, H, arPathCache);

        // Draw exit label at top-right
        const stepsLeft = arPathCache.filter(n=>n.startsWith('Node_')).length - 1;
        drawARText(ctx, W-12, 48, stepsLeft + ' rooms to exit', '#ff6b35', 13, 'right');
    }
}

function drawCanvasArrows(ctx, W, H, dir){
    // Draw 3 animated perspective arrows pointing in direction
    const cx = W/2, cy = H * 0.52;
    const t  = Date.now() / 600;  // animation time

    // Compute angle from direction emoji
    const angleMap = {
        '➡️':0,'↘️':45,'⬇️':90,'↙️':135,'⬅️':180,
        '↖️':-135,'⬆️':-90,'↗️':-45
    };
    const angleDeg = angleMap[dir.arrow] ?? 90;
    const angleRad = angleDeg * Math.PI / 180;

    // Draw 3 arrows cascading in direction
    for(let i=0;i<3;i++){
        const phase   = (t + i * 0.33) % 1;  // 0→1 animation phase
        const scale   = 0.6 + phase * 0.6;    // grows as it moves away
        const opacity = 1 - phase;            // fades out
        const dist    = phase * 120;          // travels 120px

        const ax = cx + Math.cos(angleRad) * dist;
        const ay = cy + Math.sin(angleRad) * dist;

        ctx.save();
        ctx.globalAlpha = opacity * 0.9;
        ctx.translate(ax, ay);
        ctx.rotate(angleRad + Math.PI/2);
        ctx.scale(scale, scale);

        // Arrow shape
        ctx.beginPath();
        ctx.moveTo(0, -36);
        ctx.lineTo(24, 8);
        ctx.lineTo(10, 2);
        ctx.lineTo(10, 26);
        ctx.lineTo(-10, 26);
        ctx.lineTo(-10, 2);
        ctx.lineTo(-24, 8);
        ctx.closePath();

        ctx.fillStyle = '#ff6b35';
        ctx.fill();
        ctx.strokeStyle = 'rgba(255,255,255,0.6)';
        ctx.lineWidth = 2;
        ctx.stroke();

        ctx.restore();
    }

    // Draw glow ring at center
    const grd = ctx.createRadialGradient(cx, cy, 10, cx, cy, 70);
    grd.addColorStop(0, 'rgba(255,107,53,0.25)');
    grd.addColorStop(1, 'rgba(255,107,53,0)');
    ctx.beginPath();
    ctx.arc(cx, cy, 70, 0, Math.PI*2);
    ctx.fillStyle = grd;
    ctx.fill();
}

function drawPathProgress(ctx, W, H, path){
    const rooms = path.filter(n=>n.startsWith('Node_'));
    const total = rooms.length;
    if(total < 2) return;
    const dotW  = Math.min((W - 40) / total, 36);
    const startX = (W - dotW * total) / 2;
    const y = H - 50;

    rooms.forEach((room, i)=>{
        const x = startX + i * dotW + dotW/2;
        const isExit = room.includes('Exit');
        const isCurrent = i === 0;
        ctx.beginPath();
        ctx.arc(x, y, isCurrent ? 8 : 5, 0, Math.PI*2);
        ctx.fillStyle = isExit ? '#4caf50' : (isCurrent ? '#ff6b35' : 'rgba(255,255,255,0.5)');
        ctx.fill();
        if(i < total - 1){
            ctx.beginPath();
            ctx.moveTo(x + (isCurrent?8:5), y);
            ctx.lineTo(startX + (i+1)*dotW + dotW/2 - 5, y);
            ctx.strokeStyle = 'rgba(255,255,255,0.3)';
            ctx.lineWidth = 2;
            ctx.stroke();
        }
    });

    // Label
    const label = rooms[rooms.length-1].replace('Node_Exit_','').replace(/_/g,' ');
    ctx.fillStyle = '#4caf50';
    ctx.font = 'bold 11px -apple-system,sans-serif';
    ctx.textAlign = 'center';
    ctx.fillText('→ ' + label, W/2, y + 22);
}

function drawARText(ctx, x, y, text, color, size, align='center'){
    ctx.save();
    ctx.font = 'bold '+size+'px -apple-system,sans-serif';
    ctx.textAlign = align;
    ctx.shadowColor = 'rgba(0,0,0,0.8)';
    ctx.shadowBlur  = 8;
    ctx.fillStyle = color;
    ctx.fillText(text, x, y);
    ctx.restore();
}

// When switching to AR tab, auto-start camera
document.querySelector('.tab:nth-child(3)').addEventListener('click',()=>{
    setTimeout(()=>{ if(!arRunning) startAR(); }, 200);
});

</script>
</body>
</html>`);
});

// ── HTTPS server (required for camera access on mobile) ──
const https = require('https');
const fs    = require('fs');
const path  = require('path');

// Look for certs next to server.js
const certDir = path.dirname(require.main ? require.main.filename : __filename);
const certPath = path.join(certDir, 'cert.pem');
const keyPath  = path.join(certDir, 'key.pem');

if(fs.existsSync(certPath) && fs.existsSync(keyPath)){
    const sslOptions = {
        cert: fs.readFileSync(certPath),
        key:  fs.readFileSync(keyPath),
    };
    https.createServer(sslOptions, app).listen(PORT, '0.0.0.0', ()=>{
        console.log('🔒 HTTPS Server:    https://0.0.0.0:'+PORT);
        console.log('🔒 HTTPS Dashboard: https://192.168.100.5:'+PORT+'/dashboard');
        console.log('');
        console.log('📱 Open on your phone: https://192.168.100.5:'+PORT+'/dashboard');
        console.log('   (Tap "Advanced → Proceed" on the security warning — this is safe)');
    });
} else {
    // Fallback to HTTP if certs not found
    app.listen(PORT,'0.0.0.0',()=>{
        console.log('⚠️  No SSL certs found — running HTTP (camera will NOT work on mobile)');
        console.log('🚀 Server: http://0.0.0.0:'+PORT);
        console.log('📊 Dashboard: http://0.0.0.0:'+PORT+'/dashboard');
        console.log('');
        console.log('To enable HTTPS, generate certs in the same folder as server.js:');
        console.log('  openssl req -x509 -newkey rsa:2048 -keyout key.pem -out cert.pem -days 3650 -nodes \\');
        console.log('    -subj "/CN=192.168.100.5" -addext "subjectAltName=IP:192.168.100.5,IP:127.0.0.1"');
    });
}
