#pragma once
#include <pgmspace.h>

static const char WEB_UI_HTML[] PROGMEM = R"HTML(<!doctype html>
<html lang="en-AU"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>A-OK Blind Bridge</title>
<style>
:root{--bg:#0f1115;--card:#181b22;--line:#262a33;--text:#e8eaef;--muted:#9097a3;--acc:#4f8cff;--ok:#3fbf7f;--warn:#e0a83a;--bad:#e05656}
@media (prefers-color-scheme:light){:root{--bg:#f4f5f7;--card:#fff;--line:#e2e4e9;--text:#16181d;--muted:#5f6672}}
*{box-sizing:border-box}body{margin:0;font:15px/1.45 system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--text)}
header{padding:14px 16px;display:flex;gap:10px;align-items:center;flex-wrap:wrap;border-bottom:1px solid var(--line)}
header h1{font-size:17px;margin:0 auto 0 0}.chip{font-size:12px;padding:3px 9px;border-radius:99px;background:var(--card);border:1px solid var(--line);color:var(--muted)}
.chip.ok{color:var(--ok)}.chip.bad{color:var(--bad)}.chip.warn{color:var(--warn)}
nav{display:flex;gap:4px;padding:8px 12px;overflow-x:auto;border-bottom:1px solid var(--line)}
nav button{background:none;border:0;color:var(--muted);padding:7px 11px;border-radius:8px;font:inherit;cursor:pointer;white-space:nowrap}
nav button.on{background:var(--card);color:var(--text)}
main{max-width:980px;margin:0 auto;padding:16px}section{display:none}section.on{display:block}
.grid{display:grid;grid-template-columns:repeat(auto-fill,minmax(250px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--line);border-radius:14px;padding:14px}
.card h3{margin:0 0 2px;font-size:15px}.sub{color:var(--muted);font-size:12px}
.bar{height:8px;background:var(--line);border-radius:9px;margin:12px 0 10px;overflow:hidden}.bar i{display:block;height:100%;background:var(--acc);transition:width .4s}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap}.row>*{flex:1}
button.b{background:var(--line);color:var(--text);border:0;border-radius:10px;padding:9px 10px;font:inherit;cursor:pointer;min-height:40px}
button.b:hover{filter:brightness(1.15)}button.p{background:var(--acc);color:#fff}button.d{background:var(--bad);color:#fff}
input,select{width:100%;background:var(--bg);color:var(--text);border:1px solid var(--line);border-radius:9px;padding:8px 10px;font:inherit}
input[type=range]{padding:0;accent-color:var(--acc)}input[type=checkbox]{width:auto}
label{display:block;font-size:12px;color:var(--muted);margin:10px 0 4px}
table{width:100%;border-collapse:collapse;font-size:13px}td,th{padding:7px 6px;border-bottom:1px solid var(--line);text-align:left;vertical-align:top}
.chs{display:grid;grid-template-columns:repeat(8,1fr);gap:3px}.chs label{margin:0;display:flex;gap:3px;align-items:center;color:var(--text);font-size:12px}
.note{color:var(--muted);font-size:13px;margin:6px 0 12px}.big{font-size:22px;font-weight:600}
.steps li{margin:6px 0}.done{color:var(--ok)}code{background:var(--line);padding:1px 5px;border-radius:5px}
a{color:var(--acc)}td select{min-width:92px}
#toast{position:fixed;left:50%;bottom:18px;transform:translateX(-50%);background:var(--card);border:1px solid var(--line);padding:10px 14px;border-radius:10px;display:none;max-width:90vw}
</style></head><body>
<header><h1>A-OK Blind Bridge</h1><span id="cWifi" class="chip">Wi-Fi</span><span id="cMqtt" class="chip">MQTT</span><span id="cRadio" class="chip">Radio</span><span id="cRemote" class="chip">Remote</span></header>
<nav id="nav"></nav>
<main>
<section id="s-blinds"><div id="blindCards" class="grid"></div><p id="noBlinds" class="note" style="display:none">No blinds yet. Learn your remote under <b>Remote</b>, then add blinds under <b>Set up blinds</b>.</p></section>

<section id="s-remote"><div class="card">
<h3>Learn your A-OK remote</h3><p class="note">The bridge copies your hand remote's identity, so the blinds accept it like a second remote. Nothing is changed on the motors.</p>
<ol class="steps"><li id="l1">Press <b>Start learning</b>.</li><li id="l2">Hold the remote near the bridge, pick any channel, and press <b>Up</b> once.</li><li id="l3">Press <b>Down</b> once.</li><li id="l4">Press <b>Stop</b> (the middle button) once.</li></ol>
<div class="row"><button class="b p" onclick="post('/api/remote',{action:'learn'})">Start learning</button><button class="b" onclick="post('/api/remote',{action:'cancel'})">Cancel</button><button class="b d" onclick="if(confirm('Forget the learned remote?'))post('/api/remote',{action:'forget'})">Forget remote</button></div>
<p id="learnStatus" class="note"></p>
<table id="remoteTable"></table>
<label><input type="checkbox" id="listen" onchange="saveListen()"> Follow the hand remote (when someone uses the remote, update positions in Home Assistant)</label>
</div>
<div class="card" style="margin-top:12px"><h3>Test a channel</h3><p class="note">Send a command on any remote channel before setting up blinds.</p>
<div class="row"><select id="testCh"></select><button class="b" onclick="test('open')">▲ Up</button><button class="b" onclick="test('stop')">■ Stop</button><button class="b" onclick="test('close')">▼ Down</button></div></div>
</section>

<section id="s-setup"><div class="card"><h3>Blinds</h3>
<p class="note">One row per blind (or group). Channels are the numbers on your remote's display. A blind on several channels (for example <code>1,2,3</code>) is a group: one command moves them all and its position is the average.</p>
<div style="overflow-x:auto"><table><thead><tr><th>Name</th><th>Channels</th><th>Open (s)</th><th>Close (s)</th><th>Favourite %</th><th>Type</th><th>Reversed</th><th></th></tr></thead><tbody id="editRows"></tbody></table></div>
<div class="row" style="margin-top:10px"><button class="b" onclick="addRow()">+ Add blind</button><button class="b p" onclick="saveBlinds()">Save blinds</button></div>
<p class="note">Reversed: tick if Up on the remote closes this blind. Travel times can be measured on the <b>Calibrate</b> tab.</p></div></section>

<section id="s-cal"><div class="card"><h3>Measure travel times</h3>
<ol class="steps"><li>Choose a blind and make sure it's fully open.</li><li>Press <b>Start closing</b>, and press <b>It's stopped</b> the moment it stops at the bottom.</li><li>Then press <b>Start opening</b>, and <b>It's stopped</b> when it reaches the top.</li></ol>
<label>Blind</label><select id="calBlind"></select>
<div class="row" style="margin-top:10px"><button class="b" onclick="cal('start_close')">▼ Start closing</button><button class="b" onclick="cal('start_open')">▲ Start opening</button></div>
<div class="row" style="margin-top:10px"><button class="b p" onclick="cal('done')">It's stopped</button><button class="b" onclick="cal('cancel')">Cancel</button></div>
<p class="big" id="calTimer"></p><p class="note" id="calResult"></p></div></section>

<section id="s-net"><div class="card"><h3>Wi-Fi</h3>
<label>Network name (SSID)</label><input id="n_ssid"><label>Password <span id="passSet"></span></label><input id="n_pass" type="password" placeholder="leave blank to keep">
<label>Device name</label><input id="n_hostname"><label>Web page password (optional, user <code>admin</code>)</label><input id="n_admin" type="password" placeholder="leave blank for none">
<h3 style="margin-top:18px">MQTT (Home Assistant)</h3>
<label>Broker address</label><input id="n_mqtt_host" placeholder="192.168.0.55"><label>Port</label><input id="n_mqtt_port" type="number">
<label>Username</label><input id="n_mqtt_user"><label>Password <span id="mqttPassSet"></span></label><input id="n_mqtt_pass" type="password" placeholder="leave blank to keep">
<label>Base topic</label><input id="n_base_topic"><label>Discovery prefix</label><input id="n_discovery_prefix">
<div class="row" style="margin-top:12px"><button class="b p" onclick="saveNet()">Save</button></div><p class="note">Changing Wi-Fi restarts the bridge.</p></div></section>

<section id="s-radio"><div class="card"><h3>CC1101 wiring</h3><p class="note">GPIO numbers on the ESP32. Defaults suit the HW-395 board. Saving restarts the bridge.</p>
<div class="grid" id="pinGrid"></div>
<div class="row" style="margin-top:12px"><button class="b p" onclick="savePins()">Save and restart</button></div></div>
<div class="card" style="margin-top:12px"><h3>Firmware</h3><p class="note" id="fwInfo"></p>
<input type="file" id="fw" accept=".bin"><div class="row" style="margin-top:10px"><button class="b" onclick="upload()">Upload firmware</button><button class="b" onclick="if(confirm('Restart the bridge?'))post('/api/reboot',{})">Restart</button></div></div></section>

<section id="s-log"><div class="card"><h3>Heard on 433.92 MHz</h3><p class="note">Recent A-OK frames. Ticked = your learned remote.</p><div style="overflow-x:auto"><table id="frames"></table></div></div></section>
</main><div id="toast"></div>
<script>
const tabs=[['blinds','Blinds'],['remote','Remote'],['setup','Set up blinds'],['cal','Calibrate'],['net','Network'],['radio','Radio & firmware'],['log','Activity']];
const $=id=>document.getElementById(id);let S=null,C=null,edit=null;
$('nav').innerHTML=tabs.map(t=>`<button data-t="${t[0]}" onclick="show('${t[0]}')">${t[1]}</button>`).join('');
function show(t){document.querySelectorAll('section').forEach(s=>s.classList.toggle('on',s.id=='s-'+t));document.querySelectorAll('nav button').forEach(b=>b.classList.toggle('on',b.dataset.t==t));try{localStorage.tab=t}catch(e){}if(t=='setup'||t=='net'||t=='radio')loadConfig()}
function toast(m){const t=$('toast');t.textContent=m;t.style.display='block';clearTimeout(t._h);t._h=setTimeout(()=>t.style.display='none',3500)}
async function post(u,b){try{const r=await fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});const j=await r.json().catch(()=>({}));if(!r.ok||j.ok===false){toast(j.error||('Error '+r.status));return null}refresh();return j}catch(e){toast('Bridge not reachable');return null}}
const cmd=(id,c)=>post('/api/command',{id,cmd:String(c)});
function chip(id,txt,cls){const e=$(id);e.textContent=txt;e.className='chip '+cls}
function render(){
 chip('cWifi',S.wifi.ap?'Setup Wi-Fi':S.wifi.connected?'Wi-Fi '+S.wifi.rssi+' dBm':'Wi-Fi offline',S.wifi.connected?'ok':'warn');
 chip('cMqtt',!S.mqtt.configured?'MQTT not set':S.mqtt.connected?'MQTT connected':'MQTT offline',S.mqtt.connected?'ok':S.mqtt.configured?'bad':'warn');
 chip('cRadio',S.radio.ready?'CC1101 ready':'CC1101 missing',S.radio.ready?'ok':'bad');
 chip('cRemote',S.remote.complete?'Remote '+S.remote.remote_id:S.remote.learning?'Learning…':'Remote not learned',S.remote.complete?'ok':'warn');
 const cards=$('blindCards');$('noBlinds').style.display=S.blinds.length?'none':'block';
 if(!(document.activeElement&&document.activeElement.type=='range'&&cards.contains(document.activeElement)))cards.innerHTML=S.blinds.map(b=>{const p=b.position<0?null:b.position;const st=b.motion!='stopped'?b.motion:(p==null?'position unknown':p==0?'closed':p==100?'open':p+'% open');
  return `<div class="card"><h3>${esc(b.name)}</h3><div class="sub">${b.group?'Group · ':''}channel ${b.channels} · ${st}</div>
  <div class="bar"><i style="width:${p??0}%"></i></div>
  <div class="row"><button class="b" onclick="cmd('${b.id}','OPEN')">▲</button><button class="b" onclick="cmd('${b.id}','STOP')">■</button><button class="b" onclick="cmd('${b.id}','CLOSE')">▼</button>${b.favourite>=0?`<button class="b" title="Favourite ${b.favourite}%" onclick="cmd('${b.id}','FAVOURITE')">★</button>`:''}</div>
  <input type="range" min="0" max="100" value="${p??50}" style="margin-top:12px" onchange="cmd('${b.id}',this.value);this.blur()">
  <div class="row sub" style="margin-top:6px"><a href="#" onclick="cmd('${b.id}','SYNC_OPEN');return false">Mark open</a><a href="#" onclick="cmd('${b.id}','SYNC_CLOSE');return false" style="text-align:right">Mark closed</a></div></div>`}).join('');
 const r=S.remote;$('learnStatus').textContent=r.learning?'Listening… '+r.learn_status:'Status: '+r.learn_status;
 ['open','close','stop'].forEach((k,i)=>$('l'+(i+2)).className=r[k]?'done':'');$('l1').className=r.learning||r.complete?'done':'';
 $('remoteTable').innerHTML=r.prefix&&r.prefix!='000000'?`<tr><th>Remote</th><td>${r.prefix} / ${r.remote_id}</td></tr><tr><th>Up</th><td>${r.open||'—'}</td></tr><tr><th>Down</th><td>${r.close||'—'}</td></tr><tr><th>Stop</th><td>${r.stop||'—'}</td></tr>`:'';
 if(document.activeElement!=$('listen'))$('listen').checked=r.listen;
 if(!$('testCh').options.length)$('testCh').innerHTML=Array.from({length:16},(_,i)=>`<option>${i+1}</option>`).join('');
 const cb=$('calBlind'),sel=cb.value;cb.innerHTML=S.blinds.map(b=>`<option value="${b.id}">${esc(b.name)}</option>`).join('');if(sel)cb.value=sel;
 $('calTimer').textContent=S.calibration.active?(S.calibration.direction+' '+(S.calibration.elapsed_ms/1000).toFixed(1)+' s'):'';
 $('frames').innerHTML='<tr><th></th><th>Ago</th><th>Action</th><th>Channels</th><th>Remote</th><th>Payload</th></tr>'+S.frames.map(f=>`<tr><td>${f.matched?'✓':''}</td><td>${f.ago_s}s</td><td>${f.action}${f.repeats>1?' ×'+f.repeats:''}</td><td>${f.channels}</td><td>${f.prefix}/${f.remote_id}</td><td><code>${f.payload}</code></td></tr>`).join('');
 $('fwInfo').textContent=`Version ${S.fw} · up ${Math.round(S.uptime/60)} min · free memory ${Math.round(S.heap/1024)} kB · ${S.wifi.ip}`;
}
function esc(s){return String(s).replace(/[&<>"]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;'}[c]))}
async function refresh(){try{S=await (await fetch('/api/state')).json();render()}catch(e){}}
async function loadConfig(){try{const r=await fetch('/api/config');if(!r.ok)return;C=await r.json();fillNet();fillPins();if(!edit){edit=C.blinds.map(b=>({...b}));drawRows()}}catch(e){}}
function fillNet(){const n=C.network;['ssid','hostname','mqtt_host','mqtt_port','mqtt_user','base_topic','discovery_prefix'].forEach(k=>{const e=$('n_'+k);if(document.activeElement!=e)e.value=n[k]??''});$('passSet').textContent=n.pass_set?'(saved)':'';$('mqttPassSet').textContent=n.mqtt_pass_set?'(saved)':''}
async function saveNet(){const b={};['ssid','hostname','mqtt_host','mqtt_user','base_topic','discovery_prefix'].forEach(k=>b[k]=$('n_'+k).value.trim());b.mqtt_port=parseInt($('n_mqtt_port').value)||1883;
 if($('n_pass').value)b.pass=$('n_pass').value;if($('n_mqtt_pass').value)b.mqtt_pass=$('n_mqtt_pass').value;b.admin_pass=$('n_admin').value;
 const j=await post('/api/network',b);if(j)toast(j.rebooting?'Saved — restarting and joining Wi-Fi…':'Saved')}
const pinKeys=[['sck','SCK (pin 5)'],['miso','MISO (pin 7)'],['mosi','MOSI (pin 6)'],['cs','CSN (pin 4)'],['gdo0','GDO0 (pin 3, transmit)'],['gdo2','GDO2 (pin 8, receive)'],['led','Status LED (-1 = none)'],['mhz','Frequency MHz'],['pa_dbm','Transmit power dBm']];
function fillPins(){$('pinGrid').innerHTML=pinKeys.map(([k,l])=>`<div><label>${l}</label><input id="p_${k}" value="${C.pins[k]}"></div>`).join('')}
async function savePins(){const b={};pinKeys.forEach(([k])=>b[k]=parseFloat($('p_'+k).value));const j=await post('/api/pins',b);if(j)toast('Saved — restarting')}
function drawRows(){$('editRows').innerHTML=edit.map((b,i)=>`<tr>
 <td><input value="${esc(b.name)}" oninput="edit[${i}].name=this.value"></td>
 <td style="min-width:230px"><div class="chs">${Array.from({length:16},(_,c)=>`<label><input type="checkbox" ${chHas(b.channels,c+1)?'checked':''} onchange="setCh(${i},${c+1},this.checked)">${c+1}</label>`).join('')}</div></td>
 <td><input type="number" step="0.1" value="${(b.open_ms/1000).toFixed(1)}" oninput="edit[${i}].open_ms=Math.round(this.value*1000)"></td>
 <td><input type="number" step="0.1" value="${(b.close_ms/1000).toFixed(1)}" oninput="edit[${i}].close_ms=Math.round(this.value*1000)"></td>
 <td><input type="number" min="-1" max="100" value="${b.favourite}" oninput="edit[${i}].favourite=parseInt(this.value)"></td>
 <td><select onchange="edit[${i}].device_class=this.value">${['shade','blind','curtain','awning','shutter'].map(t=>`<option ${t==b.device_class?'selected':''}>${t}</option>`).join('')}</select></td>
 <td><input type="checkbox" ${b.invert?'checked':''} onchange="edit[${i}].invert=this.checked"></td>
 <td><button class="b" onclick="edit.splice(${i},1);drawRows()">✕</button></td></tr>`).join('')}
const chList=s=>String(s||'').split(',').map(Number).filter(Boolean);const chHas=(s,c)=>chList(s).includes(c);
function setCh(i,c,on){let l=chList(edit[i].channels).filter(x=>x!=c);if(on)l.push(c);edit[i].channels=l.sort((a,b)=>a-b).join(',')}
function addRow(){const n=edit.length+1;edit.push({name:'Blind '+n,channels:String(Math.min(n,16)),open_ms:30000,close_ms:30000,favourite:-1,device_class:'shade',invert:false});drawRows()}
async function saveBlinds(){const j=await post('/api/blinds',{blinds:edit});if(j){toast('Blinds saved');edit=null;loadConfig()}}
function test(a){post('/api/test',{channels:$('testCh').value,action:a})}
async function cal(a){const j=await post('/api/calibrate',{action:a,id:$('calBlind').value});if(j&&j.measured_ms){$('calResult').textContent='Measured '+(j.measured_ms/1000).toFixed(1)+' s and saved.';edit=null}}
function saveListen(){post('/api/remote',{action:'save',remote:{listen:$('listen').checked}})}
async function upload(){const f=$('fw').files[0];if(!f)return toast('Choose a .bin file');toast('Uploading…');const fd=new FormData();fd.append('firmware',f);const r=await fetch('/update',{method:'POST',body:fd});toast(await r.text())}
let t0='blinds';try{t0=localStorage.tab||'blinds'}catch(e){}show(t0);refresh();setInterval(refresh,1200);
</script></body></html>)HTML";
