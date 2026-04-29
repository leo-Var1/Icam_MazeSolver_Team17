// =============================================================
//  web_ui.cpp — Implémentation IHM WiFi
//  Équipe 17 | ICAM Strasbourg | ESP8266 NodeMCU
// =============================================================
//
//  Architecture non-bloquante :
//  - Les handlers async s'exécutent dans un contexte interrompu.
//    Ils ne font QUE : (a) poser un flag, (b) répondre avec un court JSON.
//  - La loop() consomme les flags via web_ui_poll_cmd() et appelle
//    les fonctions nav_* dans son propre contexte (sûr).
//
//  Routes HTTP :
//    GET  /           → page HTML (PROGMEM)
//    GET  /state      → JSON état robot (polling 500ms)
//    GET  /maze       → JSON grille murs + visited
//    POST /stop       → arrêt d'urgence
//    POST /start1     → lance Run 1
//    POST /start2     → lance Run 2
//    POST /move       → body JSON {dir:"up|down|left|right"}
//    POST /target     → body JSON {row, col}
//    POST /start_pos  → body JSON {row, col}
// =============================================================

#include "web_ui.h"
#include "config.h"
#include "maze.h"
#include "calibration.h"
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <ArduinoJson.h>

// ── Instance serveur async ────────────────────────────────────
static AsyncWebServer s_server(WIFI_PORT);

// ── État partagé (écrit par main, lu par /state) ──────────────
static WebState s_state = { 0, 0.0f, 0, 0, 0, 0 };

// ── File de commande (simple flag, 1 seule en vol) ────────────
// volatile car modifié depuis un contexte async.
static volatile WebCmd s_pending_cmd    = WEB_CMD_NONE;
static volatile int    s_pending_value  = 0;  // valeur associée à la commande (ex: ticks)

// ── Target (arrivée) et Start (départ) ────────────────────────
// Par défaut : départ (0,0) coin haut-gauche, arrivée (4,4) coin bas-droit.
static volatile uint8_t s_target_row = MAZE_SIZE - 1;
static volatile uint8_t s_target_col = MAZE_SIZE - 1;
static volatile uint8_t s_start_row  = 0;
static volatile uint8_t s_start_col  = 0;
static volatile uint8_t s_start_dir  = 0;  // 0=N, 1=E, 2=S, 3=W

// =============================================================
//  Page HTML embarquée en PROGMEM
// =============================================================
static const char INDEX_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Robot Labyrinthe — Équipe 17</title>
<style>
  *{box-sizing:border-box;}
  body{font-family:system-ui,sans-serif;margin:0;padding:12px;background:#1a1a1a;color:#eee;}
  h1{font-size:1.1em;margin:0 0 10px 0;color:#aaa;}
  .wrap{display:flex;flex-wrap:wrap;gap:14px;align-items:flex-start;}
  .panel{background:#2a2a2a;padding:12px;border-radius:8px;}
  /* Grille */
  #grid{background:#111;border-radius:4px;display:block;}
  .cell{fill:#222;stroke:#333;stroke-width:1;cursor:pointer;transition:fill .15s;}
  .cell:hover{fill:#2e2e2e;}
  .cell.visited{fill:#1a3a1a;}
  .cell.target{fill:#4a3800;}
  .cell.startpos{fill:#002244;}
  .wall{stroke:#e74c3c;stroke-width:4;stroke-linecap:round;}
  .robot{fill:#3498db;}
  /* Mode selector */
  .mode-bar{display:flex;gap:6px;margin-bottom:8px;}
  .mode-btn{flex:1;padding:8px 6px;border:2px solid #555;background:#333;color:#aaa;border-radius:4px;cursor:pointer;font-size:12px;text-align:center;}
  .mode-btn.active-start{border-color:#3498db;background:#002244;color:#3498db;}
  .mode-btn.active-target{border-color:#f39c12;background:#4a3800;color:#f39c12;}
  .dir-btn{padding:4px 8px;background:#333;border:1px solid #555;color:#aaa;border-radius:3px;cursor:pointer;font-size:12px;}
  .dir-btn.active{border-color:#3498db;background:#002244;color:#3498db;}
  /* Boutons */
  .btn{background:#3a3a3a;color:#eee;border:1px solid #555;padding:9px 14px;border-radius:4px;margin:3px;cursor:pointer;font-size:13px;}
  .btn:hover:not(:disabled){background:#4a4a4a;}
  .btn.stop{background:#7a1a1a;border-color:#e74c3c;color:#f8bfbf;}
  .btn.stop:hover:not(:disabled){background:#9a2a2a;}
  .btn.run{background:#1a4a1a;border-color:#27ae60;color:#a3e9b8;}
  .btn.run:hover:not(:disabled){background:#2a5a2a;}
  .btn:disabled{opacity:.3;cursor:not-allowed;}
  .btn-row{margin-top:6px;}
  /* D-pad */
  .dpad{display:grid;grid-template-columns:repeat(3,52px);grid-template-rows:repeat(3,52px);gap:3px;margin-top:10px;}
  .dpad .btn{padding:0;font-size:18px;margin:0;width:52px;height:52px;display:flex;align-items:center;justify-content:center;}
  .dpad .up   {grid-column:2;grid-row:1;}
  .dpad .left {grid-column:1;grid-row:2;}
  .dpad .down {grid-column:2;grid-row:2;}
  .dpad .right{grid-column:3;grid-row:2;}
  /* Info */
  .info-row{font-family:monospace;font-size:13px;margin:3px 0;line-height:1.5;}
  .info-row .lbl{color:#888;margin-right:4px;}
  .tag{display:inline-block;padding:1px 7px;border-radius:3px;font-weight:bold;font-size:12px;}
  .s0{background:#27ae60;}.s1{background:#f39c12;}.s2{background:#2980b9;}
  .s3{background:#3498db;}.s4{background:#27ae60;}.s5{background:#e74c3c;}
  .tof-bar{display:grid;grid-template-columns:1fr 1fr;gap:4px;margin-top:8px;}
  .tof-val{background:#1e1e1e;padding:4px 6px;border-radius:3px;font-family:monospace;font-size:12px;text-align:center;}
  .tof-val .lbl{color:#888;display:block;font-size:10px;}
  hr{border:none;border-top:1px solid #333;margin:10px 0;}
  .hint{font-size:11px;color:#666;margin-top:5px;}
</style>
</head>
<body>
<h1>Robot Labyrinthe — Équipe 17</h1>
<div class="wrap">

  <!-- Carte labyrinthe + grille interactive -->
  <div class="panel">
    <h1 style="margin-top:0;">Carte du Labyrinthe</h1>
    <div class="mode-bar">
      <div class="mode-btn active-start" id="btn-mode-start" onclick="setMode('start')">
        Poser DEPART
      </div>
      <div class="mode-btn" id="btn-mode-target" onclick="setMode('target')">
        Poser ARRIVEE
      </div>
    </div>
    <div style="display:flex;gap:4px;margin-bottom:6px;font-size:12px;align-items:center;">
      <span style="color:#888;">Orientation départ :</span>
      <button class="dir-btn" id="dir-N" onclick="setDir(0)">▲ N</button>
      <button class="dir-btn" id="dir-E" onclick="setDir(1)">▶ E</button>
      <button class="dir-btn" id="dir-S" onclick="setDir(2)">▼ S</button>
      <button class="dir-btn" id="dir-W" onclick="setDir(3)">◀ W</button>
    </div>
    <svg id="grid" width="340" height="340" viewBox="-2 -2 344 344"></svg>
    <div id="maze-legend" style="font-size:11px;color:#888;margin-top:4px;">
      <span style="color:#1a5c1a;">&#9632;</span> Visitée &nbsp;
      <span style="color:#e74c3c;">&#9472;</span> Mur &nbsp;
      <span style="color:#3498db;">&#9650;</span> Robot &nbsp;
      <span style="color:#f39c12;">&#9632;</span> Arrivée &nbsp;
      <span style="color:#2980b9;">&#9632;</span> Départ
    </div>
    <div class="hint">Clic case = placer départ/arrivée. Carte mise à jour 1s/poll.</div>
  </div>

  <!-- Panneau de contrôle -->
  <div class="panel" style="min-width:220px;">
    <div class="info-row"><span class="lbl">État :</span><span id="s-state">—</span></div>
    <div class="info-row"><span class="lbl">Position :</span><span id="s-pos">—</span></div>
    <div class="info-row"><span class="lbl">Départ :</span><span id="s-start">—</span></div>
    <div class="info-row"><span class="lbl">Arrivée :</span><span id="s-target">—</span></div>
    <div class="info-row"><span class="lbl">Cap IMU :</span><span id="s-heading">—</span></div>
    <div class="tof-bar">
      <div class="tof-val"><span class="lbl">FL</span><span id="t-fl">—</span> mm</div>
      <div class="tof-val"><span class="lbl">FR</span><span id="t-fr">—</span> mm</div>
      <div class="tof-val"><span class="lbl">SL</span><span id="t-sl">—</span> mm</div>
      <div class="tof-val"><span class="lbl">SR</span><span id="t-sr">—</span> mm</div>
    </div>
    <hr>
    <div class="btn-row">
      <button class="btn run" id="btn-r1" onclick="cmd('start1')" style="font-size:15px;padding:12px 18px;width:100%;">▶ Lancer Tremaux (Run 1)</button>
    </div>
    <div class="btn-row">
      <button class="btn run" id="btn-r2" onclick="cmd('start2')">▶ Run 2 — BFS</button>
    </div>
    <div class="btn-row">
      <button class="btn run" id="btn-wallr" onclick="cmd('start_wall_r')"
              style="background:#1a3a4a;border-color:#3498db;color:#a3d9ec;">▶ Main Droite (fallback)</button>
    </div>
    <div class="btn-row">
      <button class="btn stop" onclick="cmd('stop')">■ STOP urgence</button>
      <button class="btn" id="btn-reset" onclick="cmd('reset')" style="display:none;background:#3a2a00;border-color:#f39c12;color:#f8d57e;">↺ Reset → IDLE</button>
    </div>
    <hr>
    <div class="hint">Manuel (IDLE) — clics ou touches clavier ↑↓←→ / Espace=stop</div>
    <div class="dpad">
      <button class="btn up"    id="dpad-up"    onclick="move('up')">▲</button>
      <button class="btn left"  id="dpad-left"  onclick="move('left')">◀</button>
      <button class="btn down"  id="dpad-down"  onclick="move('down')">▼</button>
      <button class="btn right" id="dpad-right" onclick="move('right')">▶</button>
    </div>
    <div class="btn-row" style="margin-top:10px;">
      <button class="btn" onclick="move('smooth_l')" style="background:#2c3e50;flex:1;">Smooth ◀</button>
      <button class="btn" onclick="move('smooth_r')" style="background:#2c3e50;flex:1;">Smooth ▶</button>
    </div>
  </div>

  <!-- Panneau Calibration -->
  <div class="panel" style="min-width:220px;">
    <h1 style="margin-top:0;">Calibration ToF</h1>
    <div class="info-row"><span class="lbl">CENTER  :</span><span id="cal-center">—</span> mm</div>
    <div class="info-row"><span class="lbl">TURN    :</span><span id="cal-turn">—</span> mm</div>
    <div class="info-row"><span class="lbl">OPEN L  :</span><span id="cal-ol">—</span> mm</div>
    <div class="info-row"><span class="lbl">OPEN R  :</span><span id="cal-or">—</span> mm</div>
    <div class="info-row"><span class="lbl">WALL L  :</span><span id="cal-wl">—</span> mm</div>
    <div class="info-row"><span class="lbl">WALL R  :</span><span id="cal-wr">—</span> mm</div>
    <div class="info-row"><span class="lbl">POTEAU  :</span><span id="cal-post">—</span> mm</div>
    <hr>
    <div class="btn-row">
      <button class="btn" id="btn-cal-c"   onclick="cal('center')">Capturer CENTER</button>
      <button class="btn" id="btn-cal-t"   onclick="cal('turn')">Capturer TURN</button>
      <button class="btn" id="btn-cal-ol"  onclick="cal('opening_l')">Capturer OPEN L</button>
      <button class="btn" id="btn-cal-or"  onclick="cal('opening_r')">Capturer OPEN R</button>
      <button class="btn" id="btn-cal-wl"  onclick="cal('wall_l')">Capturer MUR G</button>
      <button class="btn" id="btn-cal-wr"  onclick="cal('wall_r')">Capturer MUR D</button>
      <button class="btn" id="btn-cal-post" onclick="cal('post')">Capturer POTEAU</button>
    </div>
    <hr>
    <div style="font-size:12px;color:#888;margin-bottom:4px;">Réglages encodeurs (ticks) :</div>
    <div class="info-row" style="display:flex;align-items:center;gap:6px;">
      <span class="lbl" style="min-width:70px;">PIVOT 90°</span>
      <input id="in-piv" type="number" min="30" max="600" step="1"
             style="width:65px;background:#1e1e1e;color:#eee;border:1px solid #555;padding:3px;border-radius:3px;font-family:monospace;">
      <button class="btn" id="btn-set-piv" style="padding:4px 10px;margin:0;" onclick="setCalib('set_pivot_90','in-piv')">OK</button>
    </div>
    <div class="info-row" style="display:flex;align-items:center;gap:6px;">
      <span class="lbl" style="min-width:70px;">CASE</span>
      <input id="in-cell" type="number" min="50" max="800" step="1"
             style="width:65px;background:#1e1e1e;color:#eee;border:1px solid #555;padding:3px;border-radius:3px;font-family:monospace;">
      <button class="btn" id="btn-set-cell" style="padding:4px 10px;margin:0;" onclick="setCalib('set_cell','in-cell')">OK</button>
    </div>
    <hr>
    <div style="font-size:12px;color:#888;margin-bottom:4px;">Smooth turn (45-avance-45-centre) :</div>
    <div class="info-row" style="display:flex;align-items:center;gap:6px;">
      <span class="lbl" style="min-width:70px;">PIV 45 D</span>
      <input id="in-piv45r" type="number" min="20" max="200" step="1"
             style="width:65px;background:#1e1e1e;color:#eee;border:1px solid #555;padding:3px;border-radius:3px;font-family:monospace;">
      <button class="btn" id="btn-set-piv45r" style="padding:4px 10px;margin:0;" onclick="setCalib('set_piv45_r','in-piv45r')">OK</button>
    </div>
    <div class="info-row" style="display:flex;align-items:center;gap:6px;">
      <span class="lbl" style="min-width:70px;">PIV 45 G</span>
      <input id="in-piv45l" type="number" min="20" max="200" step="1"
             style="width:65px;background:#1e1e1e;color:#eee;border:1px solid #555;padding:3px;border-radius:3px;font-family:monospace;">
      <button class="btn" id="btn-set-piv45l" style="padding:4px 10px;margin:0;" onclick="setCalib('set_piv45_l','in-piv45l')">OK</button>
    </div>
    <div class="info-row" style="display:flex;align-items:center;gap:6px;">
      <span class="lbl" style="min-width:70px;">AVANCE</span>
      <input id="in-smove" type="number" min="20" max="400" step="1"
             style="width:65px;background:#1e1e1e;color:#eee;border:1px solid #555;padding:3px;border-radius:3px;font-family:monospace;">
      <button class="btn" id="btn-set-smove" style="padding:4px 10px;margin:0;" onclick="setCalib('set_smooth_move','in-smove')">OK</button>
      <span style="font-size:10px;color:#666;">ticks</span>
    </div>
    <div class="info-row" style="display:flex;align-items:center;gap:6px;">
      <span class="lbl" style="min-width:70px;">CENTRE</span>
      <input id="in-scenter" type="number" min="30" max="200" step="1"
             style="width:65px;background:#1e1e1e;color:#eee;border:1px solid #555;padding:3px;border-radius:3px;font-family:monospace;">
      <button class="btn" id="btn-set-scenter" style="padding:4px 10px;margin:0;" onclick="setCalib('set_smooth_center','in-scenter')">OK</button>
      <span style="font-size:10px;color:#666;">mm</span>
    </div>
    <hr>
    <div class="btn-row">
      <button class="btn stop" id="btn-cal-rst" onclick="cal('reset')">Reset defauts</button>
    </div>
    <div class="hint">IDLE uniquement. MUR G/D : robot dans couloir avec murs des 2 cotes. POTEAU : a cote d'un poteau alu. ~400ms.</div>
  </div>

  <!-- Panneau PID -->
  <div class="panel" style="min-width:300px;">
    <h1 style="margin-top:0;">Tuning PID Live</h1>
    
    <div id="pid-sync" style="font-size:10px; color:#f39c12; height:12px; margin-bottom:4px; text-align:right;"></div>

    <!-- PID Encodeurs -->
    <div style="font-size:12px;color:#3498db;margin-bottom:8px;font-weight:bold;border-bottom:1px solid #333;padding-bottom:2px;">
      PID Encodeurs (Synchro Roues)
    </div>
    <div class="pid-group">
      <div class="info-row">
        <span class="lbl" style="width:20px;">Kp</span>
        <input id="sl-kp" type="range" min="0" max="10" step="0.1" oninput="updVal('kp')" onchange="sendVal('pid_kp',this.value)">
        <span id="val-kp" class="tof-val" style="width:40px;display:inline-block">0</span>
      </div>
      <div class="info-row">
        <span class="lbl" style="width:20px;">Ki</span>
        <input id="sl-ki" type="range" min="0" max="1" step="0.005" oninput="updVal('ki')" onchange="sendVal('pid_ki',this.value)">
        <span id="val-ki" class="tof-val" style="width:40px;display:inline-block">0</span>
      </div>
      <div class="info-row">
        <span class="lbl" style="width:20px;">Kd</span>
        <input id="sl-kd" type="range" min="0" max="5" step="0.05" oninput="updVal('kd')" onchange="sendVal('pid_kd',this.value)">
        <span id="val-kd" class="tof-val" style="width:40px;display:inline-block">0</span>
      </div>
    </div>

    <hr>
    <!-- PID ToF -->
    <div style="font-size:12px;color:#27ae60;margin-bottom:8px;font-weight:bold;border-bottom:1px solid #333;padding-bottom:2px;">
      PID ToF (Centrage Murs)
    </div>
    <div class="pid-group">
      <div class="info-row">
        <span class="lbl" style="width:20px;">Kp</span>
        <input id="sl-tkp" type="range" min="0" max="2" step="0.01" oninput="updVal('tkp')" onchange="sendVal('pid_tof_kp',this.value)">
        <span id="val-tkp" class="tof-val" style="width:40px;display:inline-block">0</span>
      </div>
      <div class="info-row">
        <span class="lbl" style="width:20px;">Ki</span>
        <input id="sl-tki" type="range" min="0" max="0.1" step="0.001" oninput="updVal('tki')" onchange="sendVal('pid_tof_ki',this.value)">
        <span id="val-tki" class="tof-val" style="width:40px;display:inline-block">0</span>
      </div>
      <div class="info-row">
        <span class="lbl" style="width:20px;">Kd</span>
        <input id="sl-tkd" type="range" min="0" max="2" step="0.01" oninput="updVal('tkd')" onchange="sendVal('pid_tof_kd',this.value)">
        <span id="val-tkd" class="tof-val" style="width:40px;display:inline-block">0</span>
      </div>
    </div>

    <hr>
    <!-- Guide de réglage -->
    <div style="background:#111; padding:8px; border-radius:4px; font-size:11px; color:#aaa; line-height:1.4;">
      <b style="color:#eee;">Conseils de réglage :</b><br>
      1. <b>Kp (Réaction) :</b> Augmenter jusqu'à ce que le robot oscille, puis baisser de 30%.<br>
      2. <b>Kd (Amorti) :</b> Augmenter pour supprimer les oscillations du Kp.<br>
      3. <b>Ki (Précision) :</b> À laisser très faible (0.001). Sert à corriger une dérive lente.<br>
      <i style="color:#666; font-size:10px;">Le réglage est envoyé dès que vous relâchez le curseur.</i>
    </div>
  </div>

</div>

<script>
const N=5, SZ=64, PAD=10;
const SVG=document.getElementById('grid');
let mazeData=null, stateData=null;
let target={r:4,c:4}, startPos={r:0,c:0};
let startDir=0;  // 0=N,1=E,2=S,3=W
let mode='start';  // 'start' ou 'target'

const STATE_NAMES=['IDLE','RUN1-Trémaux','CARTE OK','RUN2-BFS','TERMINÉ','URGENCE','MAIN DROITE'];
const STATE_CLS  =['s0','s1','s2','s3','s4','s5','s1'];

function setMode(m){
  mode=m;
  document.getElementById('btn-mode-start').className =
    'mode-btn' + (m==='start' ?' active-start':'');
  document.getElementById('btn-mode-target').className =
    'mode-btn' + (m==='target'?' active-target':'');
}

function refreshDirButtons(){
  ['N','E','S','W'].forEach((k,i)=>{
    const el=document.getElementById('dir-'+k);
    if(el) el.className='dir-btn'+(i===startDir?' active':'');
  });
}
function setDir(d){
  startDir=d;
  refreshDirButtons();
  // POST sans bouger la position : on conserve startPos
  fetch('/start_pos',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({row:startPos.r,col:startPos.c,dir:d})});
}

function buildGrid(){
  let s='';
  for(let r=0;r<N;r++) for(let c=0;c<N;c++){
    const x=PAD+c*SZ, y=PAD+r*SZ;
    s+=`<rect class="cell" id="c${r}_${c}" x="${x}" y="${y}" width="${SZ}" height="${SZ}"
         onclick="cellClick(${r},${c})"/>`;
  }
  // Bordure externe
  s+=`<rect x="${PAD}" y="${PAD}" width="${N*SZ}" height="${N*SZ}"
       fill="none" stroke="#555" stroke-width="2"/>`;
  s+='<g id="walls"></g><g id="robot-g"></g>';
  SVG.innerHTML=s;
}

function cellClick(r,c){
  if(mode==='start'){
    // Efface l'ancienne case départ directement dans le DOM (sans passer par render)
    const old=document.getElementById(`c${startPos.r}_${startPos.c}`);
    if(old) old.classList.remove('startpos');
    startPos={r,c};
    const el=document.getElementById(`c${r}_${c}`);
    if(el) el.classList.add('startpos');
    fetch('/start_pos',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({row:r,col:c,dir:startDir})});
  } else {
    // Efface l'ancienne case arrivée
    const old=document.getElementById(`c${target.r}_${target.c}`);
    if(old) old.classList.remove('target');
    target={r,c};
    const el=document.getElementById(`c${r}_${c}`);
    if(el) el.classList.add('target');
    fetch('/target',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({row:r,col:c})});
  }
}

function render(){
  if(!mazeData) return;
  // Cases
  for(let r=0;r<N;r++) for(let c=0;c<N;c++){
    const el=document.getElementById(`c${r}_${c}`);
    if(!el) continue;
    el.className='cell';
    if(mazeData[r][c].v>0) el.classList.add('visited');
    if(r===target.r && c===target.c) el.classList.add('target');
    if(r===startPos.r && c===startPos.c) el.classList.add('startpos');
  }
  // Murs internes (bitmask: 1=N,2=E,4=S,8=W) — on n'affiche pas la bordure externe ici
  let w='';
  for(let r=0;r<N;r++) for(let c=0;c<N;c++){
    const m=mazeData[r][c].w;
    const x=PAD+c*SZ, y=PAD+r*SZ;
    // Mur Nord (seulement si pas bord externe)
    if((m&1) && r>0) w+=`<line class="wall" x1="${x+2}" y1="${y}" x2="${x+SZ-2}" y2="${y}"/>`;
    // Mur Est (seulement si pas bord externe)
    if((m&2) && c<N-1) w+=`<line class="wall" x1="${x+SZ}" y1="${y+2}" x2="${x+SZ}" y2="${y+SZ-2}"/>`;
    // Mur Sud
    if((m&4) && r<N-1) w+=`<line class="wall" x1="${x+2}" y1="${y+SZ}" x2="${x+SZ-2}" y2="${y+SZ}"/>`;
    // Mur Ouest
    if((m&8) && c>0) w+=`<line class="wall" x1="${x}" y1="${y+2}" x2="${x}" y2="${y+SZ-2}"/>`;
  }
  document.getElementById('walls').innerHTML=w;

  // Robot
  if(stateData){
    const pr=stateData.pos.row, pc=stateData.pos.col, pf=stateData.pos.facing;
    const rx=PAD+pc*SZ+SZ/2, ry=PAD+pr*SZ+SZ/2;
    document.getElementById('robot-g').innerHTML=
      `<g transform="translate(${rx},${ry}) rotate(${pf*90})">
         <polygon class="robot" points="0,-20 13,13 0,5 -13,13" opacity="0.9"/>
       </g>`;
  }
}

function updateInfo(st){
  if(!st) return;
  const i=st.state;
  document.getElementById('s-state').innerHTML=
    `<span class="tag ${STATE_CLS[i]||''}">${STATE_NAMES[i]||'?'}</span>`;
  document.getElementById('s-pos').textContent=
    `(${st.pos.row}, ${st.pos.col})  ${'NESW'[st.pos.facing]||'?'}`;
  document.getElementById('s-heading').textContent=st.heading.toFixed(1)+'°';
  document.getElementById('t-fl').textContent=st.tof.fl;
  document.getElementById('t-fr').textContent=st.tof.fr;
  document.getElementById('t-sl').textContent=st.tof.sl;
  document.getElementById('t-sr').textContent=st.tof.sr;

  // Sync target/start depuis server si dispo
  if(st.target){ target={r:st.target.row, c:st.target.col}; }
  if(st.startpos){
    startPos={r:st.startpos.row, c:st.startpos.col};
    if(typeof st.startpos.dir==='number' && st.startpos.dir!==startDir){
      startDir=st.startpos.dir;
      refreshDirButtons();
    }
  }
  document.getElementById('s-start').textContent=`(${startPos.r}, ${startPos.c})`;
  document.getElementById('s-target').textContent=`(${target.r}, ${target.c})`;

  // Run1 accessible depuis IDLE, MAZE_OK ou EMERGENCY (relance)
  const canRun1 = (i===0||i===2||i===5);
  document.getElementById('btn-r1').disabled = !canRun1;
  document.getElementById('btn-r2').disabled = (i!==2);
  // Main droite : depuis IDLE ou EMERGENCY
  const canWallR = (i===0||i===5);
  const bWR = document.getElementById('btn-wallr');
  if(bWR) bWR.disabled = !canWallR;
  // Bouton Reset visible uniquement en EMERGENCY (state 5)
  document.getElementById('btn-reset').style.display = (i===5)?'inline-block':'none';

  // Calibration : uniquement en IDLE (state 0)
  const idle = (i===0);
  ['btn-cal-c','btn-cal-t','btn-cal-ol','btn-cal-or','btn-cal-wl','btn-cal-wr','btn-cal-post','btn-cal-rst',
   'btn-set-piv','btn-set-cell','btn-set-piv45r','btn-set-piv45l','btn-set-smove','btn-set-scenter']
    .forEach(id => { const el=document.getElementById(id); if(el) el.disabled = !idle; });
}

function cmd(what){ fetch('/'+what,{method:'POST'}); }
function cal(what){ fetch('/calib/'+what,{method:'POST'}); }
function setCalib(endpoint, inputId){
  const v=parseInt(document.getElementById(inputId).value,10);
  if(isNaN(v)) return;
  fetch('/calib/'+endpoint,{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({v})});
}

function updVal(id) {
  document.getElementById('val-'+id).textContent = document.getElementById('sl-'+id).value;
}

async function sendVal(endpoint, val) {
  const sync = document.getElementById('pid-sync');
  sync.textContent = 'Sync...';
  try {
    const r = await fetch('/calib/set_'+endpoint, {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body:JSON.stringify({v: parseFloat(val)})
    });
    if (r.ok) {
      sync.textContent = 'OK';
      setTimeout(() => { if(sync.textContent==='OK') sync.textContent=''; }, 1000);
    } else {
      sync.textContent = 'Erreur';
    }
  } catch(e) {
    sync.textContent = 'Erreur réseau';
  }
}

async function pollCalib(){
  try{
    const c=await fetch('/calib').then(r=>r.json());
    document.getElementById('cal-center').textContent=c.center;
    document.getElementById('cal-turn').textContent=c.turn;
    document.getElementById('cal-ol').textContent=c.opening_l;
    document.getElementById('cal-or').textContent=c.opening_r;
    document.getElementById('cal-wl').textContent=c.wall_l;
    document.getElementById('cal-wr').textContent=c.wall_r;
    document.getElementById('cal-post').textContent=c.post_detect;

    const setIfIdle=(id,val)=>{
      const el=document.getElementById(id);
      if(el && document.activeElement!==el) el.value=val;
    };
    const setSl=(id,val)=>{
      const sl=document.getElementById('sl-'+id);
      if(sl && document.activeElement!==sl) {
        sl.value=val;
        const vdisplay = document.getElementById('val-'+id);
        if(vdisplay) vdisplay.textContent=val;
      }
    };

    setIfIdle('in-piv',     c.pivot_90_ticks);
    setIfIdle('in-cell',    c.cell_ticks);
    setIfIdle('in-piv45r',  c.pivot_45_r);
    setIfIdle('in-piv45l',  c.pivot_45_l);
    setIfIdle('in-smove',   c.smooth_move);
    setIfIdle('in-scenter', c.smooth_center);

    // Sync sliders
    setSl('kp',  c.pid_kp);
    setSl('ki',  c.pid_ki);
    setSl('kd',  c.pid_kd);
    setSl('tkp', c.pid_tof_kp);
    setSl('tki', c.pid_tof_ki);
    setSl('tkd', c.pid_tof_kd);
  } catch(e){ console.warn('calib poll',e); }
}

function move(dir){
  fetch('/move',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({dir})});
}

// State : 500ms — position, capteurs, ToF (change vite)
async function pollState(){
  try{
    stateData=await fetch('/state').then(r=>r.json());
    updateInfo(stateData);
    render();  // re-dessine la position robot à chaque poll état
  } catch(e){ console.warn('state poll',e); }
}

// Maze : 1000ms — carte des murs (change lentement)
async function pollMaze(){
  try{
    const mz=await fetch('/maze').then(r=>r.json());
    mazeData=mz.cells;
    render();
  } catch(e){ console.warn('maze poll',e); }
}

// ── Raccourcis clavier ────────────────────────────────────────
// Flèches = contrôle manuel, Espace = stop urgence, T = lancer Trémaux
document.addEventListener('keydown', function(e){
  if(e.target.tagName==='INPUT'||e.target.tagName==='TEXTAREA') return;
  switch(e.key){
    case 'ArrowUp':    e.preventDefault(); move('up');    flash('dpad-up');    break;
    case 'ArrowDown':  e.preventDefault(); move('down');  flash('dpad-down');  break;
    case 'ArrowLeft':  e.preventDefault(); move('left');  flash('dpad-left');  break;
    case 'ArrowRight': e.preventDefault(); move('right'); flash('dpad-right'); break;
    case ' ':          e.preventDefault(); cmd('stop');   break;
    case 't': case 'T':
      if(!document.getElementById('btn-r1').disabled){ cmd('start1'); }
      break;
  }
});
function flash(id){
  const el=document.getElementById(id);
  if(!el) return;
  el.style.background='#555';
  setTimeout(()=>el.style.background='',150);
}

buildGrid();
refreshDirButtons();
pollState();
pollMaze();
setInterval(pollState,500);
setInterval(pollMaze,1000);
pollCalib();
setInterval(pollCalib,1000);
</script>
</body>
</html>
)HTML";

// =============================================================
//  Helpers communs body handler
// =============================================================
static bool parse_row_col(uint8_t* data, size_t len, uint8_t& row, uint8_t& col,
                           AsyncWebServerRequest* req) {
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return false;
    }
    int r = doc["row"] | -1;
    int c = doc["col"] | -1;
    if (r < 0 || r >= MAZE_SIZE || c < 0 || c >= MAZE_SIZE) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
        return false;
    }
    row = (uint8_t)r;
    col = (uint8_t)c;
    return true;
}

// =============================================================
//  Handlers HTTP
// =============================================================

// GET /state
static void handle_state(AsyncWebServerRequest* req) {
    StaticJsonDocument<384> doc;
    doc["state"]   = s_state.robot_state;
    doc["heading"] = s_state.heading;

    JsonObject tof = doc.createNestedObject("tof");
    tof["fl"] = s_state.tof_fl;
    tof["fr"] = s_state.tof_fr;
    tof["sl"] = s_state.tof_sl;
    tof["sr"] = s_state.tof_sr;

    JsonObject pos = doc.createNestedObject("pos");
    pos["row"]    = maze_get_row();
    pos["col"]    = maze_get_col();
    pos["facing"] = maze_get_dir();

    JsonObject tgt = doc.createNestedObject("target");
    tgt["row"] = s_target_row;
    tgt["col"] = s_target_col;

    JsonObject sp = doc.createNestedObject("startpos");
    sp["row"] = s_start_row;
    sp["col"] = s_start_col;
    sp["dir"] = s_start_dir;

    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
}

// GET /maze
static void handle_maze(AsyncWebServerRequest* req) {
    StaticJsonDocument<1024> doc;
    JsonArray rows = doc.createNestedArray("cells");
    for (uint8_t r = 0; r < MAZE_SIZE; ++r) {
        JsonArray row = rows.createNestedArray();
        for (uint8_t c = 0; c < MAZE_SIZE; ++c) {
            JsonObject cell = row.createNestedObject();
            cell["w"] = maze[r][c].walls;
            cell["v"] = maze[r][c].visited;
        }
    }
    String out;
    serializeJson(doc, out);
    req->send(200, "application/json", out);
}

// POST /stop
static void handle_stop(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_STOP;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /reset → retour IDLE depuis EMERGENCY
static void handle_reset(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_RESET_IDLE;
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /start1
static void handle_start1(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_START1;
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /start2
static void handle_start2(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_START2;
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /start_wall_r — lance l'algo main droite
static void handle_start_wall_r(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_START_WALL_R;
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /move
static void handle_move_body(AsyncWebServerRequest* req, uint8_t* data, size_t len,
                             size_t index, size_t total) {
    if (index != 0 || len != total) return;
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
    }
    const char* dir = doc["dir"] | "";
    if      (!strcmp(dir, "up"))    s_pending_cmd = WEB_CMD_MOVE_UP;
    else if (!strcmp(dir, "down"))  s_pending_cmd = WEB_CMD_MOVE_DOWN;
    else if (!strcmp(dir, "left"))  s_pending_cmd = WEB_CMD_MOVE_LEFT;
    else if (!strcmp(dir, "right")) s_pending_cmd = WEB_CMD_MOVE_RIGHT;
    else if (!strcmp(dir, "smooth_l")) s_pending_cmd = WEB_CMD_SMOOTH_L;
    else if (!strcmp(dir, "smooth_r")) s_pending_cmd = WEB_CMD_SMOOTH_R;
    else {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"dir\"}");
        return;
    }
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /target
static void handle_target_body(AsyncWebServerRequest* req, uint8_t* data, size_t len,
                               size_t index, size_t total) {
    if (index != 0 || len != total) return;
    uint8_t r, c;
    if (!parse_row_col(data, len, r, c, req)) return;
    s_target_row = r;
    s_target_col = c;
    Serial.print("[WEB] Arrivée → ("); Serial.print(r);
    Serial.print(","); Serial.print(c); Serial.println(")");
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /start_pos — body {row, col, dir?} (dir optionnel : 0=N,1=E,2=S,3=W)
static void handle_start_pos_body(AsyncWebServerRequest* req, uint8_t* data, size_t len,
                                  size_t index, size_t total) {
    if (index != 0 || len != total) return;
    StaticJsonDocument<96> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
    }
    int r = doc["row"] | -1;
    int c = doc["col"] | -1;
    int d = doc["dir"] | (int)s_start_dir;  // si absent : conserve la valeur actuelle
    if (r < 0 || r >= MAZE_SIZE || c < 0 || c >= MAZE_SIZE || d < 0 || d > 3) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
        return;
    }
    s_start_row = (uint8_t)r;
    s_start_col = (uint8_t)c;
    s_start_dir = (uint8_t)d;
    maze_set_pos((uint8_t)r, (uint8_t)c, (uint8_t)d);
    Serial.print("[WEB] Départ → ("); Serial.print(r);
    Serial.print(","); Serial.print(c);
    Serial.print(") facing="); Serial.println("NESW"[d]);
    req->send(200, "application/json", "{\"ok\":true}");
}

// =============================================================
//  Handlers HTTP — Calibration
// =============================================================

// GET /calib → JSON des seuils courants
static void handle_calib_get(AsyncWebServerRequest* req) {
    StaticJsonDocument<512> doc;
    doc["center"]         = calib_get_center();
    doc["turn"]           = calib_get_turn();
    doc["opening_l"]      = calib_get_opening_l();
    doc["opening_r"]      = calib_get_opening_r();
    doc["post_detect"]    = calib_get_post_detect();
    doc["wall_l"]         = calib_get_wall_l();
    doc["wall_r"]         = calib_get_wall_r();
    doc["pivot_90_ticks"] = calib_get_pivot_90_ticks();
    doc["cell_ticks"]     = calib_get_cell_ticks();
    doc["pivot_45_r"]     = calib_get_pivot_45_r();
    doc["pivot_45_l"]     = calib_get_pivot_45_l();
    doc["smooth_move"]    = calib_get_smooth_move();
    doc["smooth_center"]  = calib_get_smooth_center();
    doc["pid_kp"]         = calib_get_pid_kp();
    doc["pid_ki"]         = calib_get_pid_ki();
    doc["pid_kd"]         = calib_get_pid_kd();
    doc["pid_tof_kp"]     = calib_get_pid_tof_kp();
    doc["pid_tof_ki"]     = calib_get_pid_tof_ki();
    doc["pid_tof_kd"]     = calib_get_pid_tof_kd();
    doc["pid_tof_max_corr"] = calib_get_pid_tof_max_corr();
    doc["pid_piv_kp"]     = calib_get_pid_piv_kp();
    doc["pid_piv_ki"]     = calib_get_pid_piv_ki();
    doc["pid_piv_kd"]     = calib_get_pid_piv_kd();
    doc["pid_piv_max_corr"] = calib_get_pid_piv_max_corr();
    String out; serializeJson(doc, out);
    req->send(200, "application/json", out);
}

// POST /calib/center → demande capture distance centrage
static void handle_calib_center(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_CENTER;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/turn → demande capture distance d'arrêt frontale
static void handle_calib_turn(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_TURN;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/opening_l → demande capture seuil passage gauche
static void handle_calib_opening_l(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_OPENING_L;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/opening_r → demande capture seuil passage droit
static void handle_calib_opening_r(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_OPENING_R;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/reset → remet les défauts config.h
static void handle_calib_reset(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_RESET;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/post → capture seuil poteau alu
static void handle_calib_post(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_POST;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/wall_l → capture distance mur gauche
static void handle_calib_wall_l(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_WALL_L;
    req->send(200, "application/json", "{\"ok\":true}");
}
// POST /calib/wall_r → capture distance mur droit
static void handle_calib_wall_r(AsyncWebServerRequest* req) {
    s_pending_cmd = WEB_CMD_CALIB_WALL_R;
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /calib/set_pivot_90 — body {v: ticks}
static void handle_calib_set_pivot_90_body(AsyncWebServerRequest* req, uint8_t* data,
                                            size_t len, size_t index, size_t total) {
    if (index != 0 || len != total) return;
    StaticJsonDocument<48> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
    }
    int v = doc["v"] | -1;
    if (v < 30 || v > 600) {  // garde-fous (extrêmes plausibles)
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
        return;
    }
    s_pending_value = v;
    s_pending_cmd   = WEB_CMD_CALIB_SET_PIVOT_90;
    req->send(200, "application/json", "{\"ok\":true}");
}

// POST /calib/set_cell — body {v: ticks}
static void handle_calib_set_cell_body(AsyncWebServerRequest* req, uint8_t* data,
                                        size_t len, size_t index, size_t total) {
    if (index != 0 || len != total) return;
    StaticJsonDocument<48> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
    }
    int v = doc["v"] | -1;
    if (v < 50 || v > 800) {  // garde-fous
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
        return;
    }
    s_pending_value = v;
    s_pending_cmd   = WEB_CMD_CALIB_SET_CELL;
    req->send(200, "application/json", "{\"ok\":true}");
}

// Helper factorisé : parse {v} dans [vmin..vmax], pose cmd + valeur
static void handle_set_int_body(AsyncWebServerRequest* req, uint8_t* data,
                                 size_t len, size_t index, size_t total,
                                 int vmin, int vmax, WebCmd cmd) {
    if (index != 0 || len != total) return;
    StaticJsonDocument<48> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
    }
    int v = doc["v"] | INT32_MIN;
    if (v < vmin || v > vmax) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
        return;
    }
    s_pending_value = v;
    s_pending_cmd   = cmd;
    req->send(200, "application/json", "{\"ok\":true}");
}

static void handle_set_piv45_r_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_int_body(req, d, l, i, t, 20, 200, WEB_CMD_CALIB_SET_PIV45_R);
}
static void handle_set_piv45_l_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_int_body(req, d, l, i, t, 20, 200, WEB_CMD_CALIB_SET_PIV45_L);
}
static void handle_set_smooth_move_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_int_body(req, d, l, i, t, 20, 400, WEB_CMD_CALIB_SET_SMOOTH_MOVE);
}
static void handle_set_smooth_center_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_int_body(req, d, l, i, t, 30, 200, WEB_CMD_CALIB_SET_SMOOTH_CENTER);
}

// Helper pour les floats : on multiplie par 1000 pour passer dans l'int s_pending_value
static void handle_set_float_body(AsyncWebServerRequest* req, uint8_t* data,
                                 size_t len, size_t index, size_t total,
                                 float vmin, float vmax, WebCmd cmd) {
    if (index != 0 || len != total) return;
    StaticJsonDocument<64> doc;
    if (deserializeJson(doc, data, len)) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"json\"}");
        return;
    }
    float v = doc["v"] | -1000.0f;
    if (v < vmin || v > vmax) {
        req->send(400, "application/json", "{\"ok\":false,\"err\":\"range\"}");
        return;
    }
    s_pending_value = (int)(v * 1000.0f);
    s_pending_cmd   = cmd;
    req->send(200, "application/json", "{\"ok\":true}");
}

static void handle_set_pid_kp_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 20.0f, WEB_CMD_CALIB_SET_PID_KP);
}
static void handle_set_pid_ki_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 5.0f, WEB_CMD_CALIB_SET_PID_KI);
}
static void handle_set_pid_kd_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 10.0f, WEB_CMD_CALIB_SET_PID_KD);
}
static void handle_set_pid_tof_kp_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 5.0f, WEB_CMD_CALIB_SET_PID_TOF_KP);
}
static void handle_set_pid_tof_ki_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 1.0f, WEB_CMD_CALIB_SET_PID_TOF_KI);
}
static void handle_set_pid_tof_kd_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 5.0f, WEB_CMD_CALIB_SET_PID_TOF_KD);
}
static void handle_set_pid_tof_max_corr_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_int_body(req, d, l, i, t, 0, 100, WEB_CMD_CALIB_SET_PID_TOF_MAX_CORR);
}
static void handle_set_pid_piv_kp_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 20.0f, WEB_CMD_CALIB_SET_PID_PIV_KP);
}
static void handle_set_pid_piv_ki_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 5.0f, WEB_CMD_CALIB_SET_PID_PIV_KI);
}
static void handle_set_pid_piv_kd_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_float_body(req, d, l, i, t, 0.0f, 10.0f, WEB_CMD_CALIB_SET_PID_PIV_KD);
}
static void handle_set_pid_piv_max_corr_body(AsyncWebServerRequest* req, uint8_t* d, size_t l, size_t i, size_t t) {
    handle_set_int_body(req, d, l, i, t, 0, 100, WEB_CMD_CALIB_SET_PID_PIV_MAX_CORR);
}

// =============================================================
//  web_ui_init
// =============================================================
void web_ui_init() {
    WiFi.mode(WIFI_AP);
    bool ok = WiFi.softAP(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WEB] AP "); Serial.print(WIFI_SSID);
    Serial.print(ok ? " OK  IP=" : " ERREUR  IP=");
    Serial.println(WiFi.softAPIP());

    s_server.on("/", HTTP_GET, [](AsyncWebServerRequest* req) {
        req->send_P(200, "text/html", INDEX_HTML);
    });
    s_server.on("/state",  HTTP_GET,  handle_state);
    s_server.on("/maze",   HTTP_GET,  handle_maze);
    s_server.on("/stop",   HTTP_POST, handle_stop);
    s_server.on("/reset",  HTTP_POST, handle_reset);
    s_server.on("/start1", HTTP_POST, handle_start1);
    s_server.on("/start2", HTTP_POST, handle_start2);
    s_server.on("/start_wall_r", HTTP_POST, handle_start_wall_r);

    // Routes calibration
    s_server.on("/calib",            HTTP_GET,  handle_calib_get);
    s_server.on("/calib/center",     HTTP_POST, handle_calib_center);
    s_server.on("/calib/turn",       HTTP_POST, handle_calib_turn);
    s_server.on("/calib/opening_l",  HTTP_POST, handle_calib_opening_l);
    s_server.on("/calib/opening_r",  HTTP_POST, handle_calib_opening_r);
    s_server.on("/calib/reset",      HTTP_POST, handle_calib_reset);
    s_server.on("/calib/post",       HTTP_POST, handle_calib_post);
    s_server.on("/calib/wall_l",     HTTP_POST, handle_calib_wall_l);
    s_server.on("/calib/wall_r",     HTTP_POST, handle_calib_wall_r);
    s_server.on("/calib/set_pivot_90", HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_calib_set_pivot_90_body);
    s_server.on("/calib/set_cell",     HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_calib_set_cell_body);
    s_server.on("/calib/set_piv45_r",  HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_set_piv45_r_body);
    s_server.on("/calib/set_piv45_l",  HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_set_piv45_l_body);
    s_server.on("/calib/set_smooth_move",   HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_set_smooth_move_body);
    s_server.on("/calib/set_smooth_center", HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_set_smooth_center_body);

    // PID Tuning routes
    s_server.on("/calib/set_pid_kp", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_kp_body);
    s_server.on("/calib/set_pid_ki", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_ki_body);
    s_server.on("/calib/set_pid_kd", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_kd_body);
    s_server.on("/calib/set_pid_tof_kp", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_tof_kp_body);
    s_server.on("/calib/set_pid_tof_ki", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_tof_ki_body);
    s_server.on("/calib/set_pid_tof_kd", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_tof_kd_body);
    s_server.on("/calib/set_pid_tof_max_corr", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_tof_max_corr_body);
    s_server.on("/calib/set_pid_piv_kp", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_piv_kp_body);
    s_server.on("/calib/set_pid_piv_ki", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_piv_ki_body);
    s_server.on("/calib/set_pid_piv_kd", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_piv_kd_body);
    s_server.on("/calib/set_pid_piv_max_corr", HTTP_POST, [](AsyncWebServerRequest* req){}, nullptr, handle_set_pid_piv_max_corr_body);

    // Routes avec body JSON
    s_server.on("/move", HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_move_body);
    s_server.on("/target", HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_target_body);
    s_server.on("/start_pos", HTTP_POST,
        [](AsyncWebServerRequest* req){}, nullptr, handle_start_pos_body);

    s_server.onNotFound([](AsyncWebServerRequest* req) {
        req->send(404, "text/plain", "404");
    });

    s_server.begin();
    Serial.println("[WEB] Serveur démarré sur port 80");
}

// =============================================================
//  API publique
// =============================================================
void web_ui_set_state(const WebState& st) {
    s_state = st;
}

WebCmd web_ui_poll_cmd() {
    WebCmd c = (WebCmd)s_pending_cmd;
    s_pending_cmd = WEB_CMD_NONE;
    return c;
}

int web_ui_last_value() {
    return (int)s_pending_value;
}

uint8_t web_ui_get_target_row() { return s_target_row; }
uint8_t web_ui_get_target_col() { return s_target_col; }
uint8_t web_ui_get_start_row()  { return s_start_row; }
uint8_t web_ui_get_start_col()  { return s_start_col; }
uint8_t web_ui_get_start_dir()  { return s_start_dir; }
