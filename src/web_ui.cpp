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
static volatile WebCmd s_pending_cmd = WEB_CMD_NONE;

// ── Target (arrivée) et Start (départ) ────────────────────────
// Par défaut : départ (0,0) coin haut-gauche, arrivée (4,4) coin bas-droit.
static volatile uint8_t s_target_row = MAZE_SIZE - 1;
static volatile uint8_t s_target_col = MAZE_SIZE - 1;
static volatile uint8_t s_start_row  = 0;
static volatile uint8_t s_start_col  = 0;

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

  <!-- Grille SVG -->
  <div class="panel">
    <div class="mode-bar">
      <div class="mode-btn active-start" id="btn-mode-start" onclick="setMode('start')">
        🟦 Poser DÉPART
      </div>
      <div class="mode-btn" id="btn-mode-target" onclick="setMode('target')">
        🟨 Poser ARRIVÉE
      </div>
    </div>
    <svg id="grid" width="340" height="340" viewBox="-2 -2 344 344"></svg>
    <div class="hint">Clique sur une case pour placer le départ ou l'arrivée selon le mode sélectionné.</div>
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
      <button class="btn run" id="btn-r1" onclick="cmd('start1')">▶ Run 1 — Trémaux</button>
      <button class="btn run" id="btn-r2" onclick="cmd('start2')">▶ Run 2 — BFS</button>
    </div>
    <div class="btn-row">
      <button class="btn stop" onclick="cmd('stop')">■ STOP urgence</button>
    </div>
    <hr>
    <div class="hint">Contrôle manuel (IDLE uniquement)</div>
    <div class="dpad">
      <button class="btn up"    onclick="move('up')">▲</button>
      <button class="btn left"  onclick="move('left')">◀</button>
      <button class="btn down"  onclick="move('down')">▼</button>
      <button class="btn right" onclick="move('right')">▶</button>
    </div>
  </div>

  <!-- Panneau Calibration -->
  <div class="panel" style="min-width:220px;">
    <h1 style="margin-top:0;">Calibration ToF</h1>
    <div class="info-row"><span class="lbl">CENTER  :</span><span id="cal-center">—</span> mm</div>
    <div class="info-row"><span class="lbl">TURN    :</span><span id="cal-turn">—</span> mm</div>
    <div class="info-row"><span class="lbl">OPEN L  :</span><span id="cal-ol">—</span> mm</div>
    <div class="info-row"><span class="lbl">OPEN R  :</span><span id="cal-or">—</span> mm</div>
    <hr>
    <div class="btn-row">
      <button class="btn" id="btn-cal-c"  onclick="cal('center')">Capturer CENTER</button>
      <button class="btn" id="btn-cal-t"  onclick="cal('turn')">Capturer TURN</button>
      <button class="btn" id="btn-cal-ol" onclick="cal('opening_l')">Capturer OPEN L</button>
      <button class="btn" id="btn-cal-or" onclick="cal('opening_r')">Capturer OPEN R</button>
    </div>
    <div class="btn-row">
      <button class="btn stop" id="btn-cal-rst" onclick="cal('reset')">Reset defauts</button>
    </div>
    <div class="hint">IDLE uniquement. Placer le robot puis cliquer. ~400ms de mesure.</div>
  </div>

</div>

<script>
const N=5, SZ=64, PAD=10;
const SVG=document.getElementById('grid');
let mazeData=null, stateData=null;
let target={r:4,c:4}, startPos={r:0,c:0};
let mode='start';  // 'start' ou 'target'

const STATE_NAMES=['IDLE','RUN1-Trémaux','CARTE OK','RUN2-BFS','TERMINÉ','URGENCE'];
const STATE_CLS  =['s0','s1','s2','s3','s4','s5'];

function setMode(m){
  mode=m;
  document.getElementById('btn-mode-start').className =
    'mode-btn' + (m==='start' ?' active-start':'');
  document.getElementById('btn-mode-target').className =
    'mode-btn' + (m==='target'?' active-target':'');
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
          body:JSON.stringify({row:r,col:c})});
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
  if(st.startpos){ startPos={r:st.startpos.row, c:st.startpos.col}; }
  document.getElementById('s-start').textContent=`(${startPos.r}, ${startPos.c})`;
  document.getElementById('s-target').textContent=`(${target.r}, ${target.c})`;

  // Griser run2 si pas en IDLE/MAZE_OK
  const canRun = (i===0||i===2);
  document.getElementById('btn-r1').disabled = !canRun;
  document.getElementById('btn-r2').disabled = (i!==2);

  // Calibration : uniquement en IDLE (state 0)
  const idle = (i===0);
  ['btn-cal-c','btn-cal-t','btn-cal-ol','btn-cal-or','btn-cal-rst']
    .forEach(id => document.getElementById(id).disabled = !idle);
}

function cmd(what){ fetch('/'+what,{method:'POST'}); }
function cal(what){ fetch('/calib/'+what,{method:'POST'}); }

async function pollCalib(){
  try{
    const c=await fetch('/calib').then(r=>r.json());
    document.getElementById('cal-center').textContent=c.center;
    document.getElementById('cal-turn').textContent=c.turn;
    document.getElementById('cal-ol').textContent=c.opening_l;
    document.getElementById('cal-or').textContent=c.opening_r;
  } catch(e){ console.warn('calib poll',e); }
}

function move(dir){
  fetch('/move',{method:'POST',headers:{'Content-Type':'application/json'},
        body:JSON.stringify({dir})});
}

async function poll(){
  try{
    const [st,mz]=await Promise.all([
      fetch('/state').then(r=>r.json()),
      fetch('/maze').then(r=>r.json())
    ]);
    stateData=st;
    mazeData=mz.cells;
    updateInfo(st);
    render();
  } catch(e){ console.warn('poll error',e); }
}

buildGrid();
poll();
setInterval(poll,500);
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

// POST /start_pos
static void handle_start_pos_body(AsyncWebServerRequest* req, uint8_t* data, size_t len,
                                  size_t index, size_t total) {
    if (index != 0 || len != total) return;
    uint8_t r, c;
    if (!parse_row_col(data, len, r, c, req)) return;
    s_start_row = r;
    s_start_col = c;
    // Met à jour la position robot dans le labyrinthe (conserve le facing actuel)
    maze_set_pos(r, c, maze_get_dir());
    Serial.print("[WEB] Départ → ("); Serial.print(r);
    Serial.print(","); Serial.print(c); Serial.println(")");
    req->send(200, "application/json", "{\"ok\":true}");
}

// =============================================================
//  Handlers HTTP — Calibration
// =============================================================

// GET /calib → JSON des 4 seuils courants
static void handle_calib_get(AsyncWebServerRequest* req) {
    StaticJsonDocument<128> doc;
    doc["center"]    = calib_get_center();
    doc["turn"]      = calib_get_turn();
    doc["opening_l"] = calib_get_opening_l();
    doc["opening_r"] = calib_get_opening_r();
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
    s_server.on("/start1", HTTP_POST, handle_start1);
    s_server.on("/start2", HTTP_POST, handle_start2);

    // Routes calibration
    s_server.on("/calib",            HTTP_GET,  handle_calib_get);
    s_server.on("/calib/center",     HTTP_POST, handle_calib_center);
    s_server.on("/calib/turn",       HTTP_POST, handle_calib_turn);
    s_server.on("/calib/opening_l",  HTTP_POST, handle_calib_opening_l);
    s_server.on("/calib/opening_r",  HTTP_POST, handle_calib_opening_r);
    s_server.on("/calib/reset",      HTTP_POST, handle_calib_reset);

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

uint8_t web_ui_get_target_row() { return s_target_row; }
uint8_t web_ui_get_target_col() { return s_target_col; }
uint8_t web_ui_get_start_row()  { return s_start_row; }
uint8_t web_ui_get_start_col()  { return s_start_col; }
