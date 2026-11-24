// Strona sterowania serwami serwowana przez Cardputera (GET /).
// Dziala na telefonie/tablecie (dotyk, multitouch - dwa kciuki = dwa serwa)
// i na komputerze (mysz, kolko myszy, te same klawisze co na Cardputerze).
// Komunikacja: WebSocket na porcie 81, tekstowe komendy (patrz main.cpp: handleNetCmd).
#pragma once
#include <Arduino.h>

static const char WEB_UI[] PROGMEM = R"HTML(<!doctype html>
<html lang="pl">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover">
<meta name="theme-color" content="#0a0e16">
<meta name="mobile-web-app-capable" content="yes">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>Servo Control</title>
<style>
:root{--bg:#0a0e16;--panel:#181e2c;--track:#283042;--acc:#ff961e;--idle:#5aa0e6;--off:#5a5a64;
--text:#ebeef5;--dim:#7882a0;--ok:#46d26e;--bad:#eb4646;--r:14px}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;background:var(--bg);color:var(--text);
font:15px/1.35 system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
body{padding:max(10px,env(safe-area-inset-top)) max(10px,env(safe-area-inset-right))
 max(14px,env(safe-area-inset-bottom)) max(10px,env(safe-area-inset-left))}
main{max-width:980px;margin:0 auto}
header{display:flex;align-items:center;gap:10px;margin:2px 2px 10px}
h1{font-size:17px;letter-spacing:.08em;margin:0;flex:1}
.pill{font-size:12px;font-weight:600;padding:4px 9px;border-radius:99px;background:var(--panel);color:var(--dim);white-space:nowrap}
.pill.ok{color:var(--ok)}.pill.bad{color:var(--bad)}
#conn::before{content:"";display:inline-block;width:8px;height:8px;border-radius:50%;background:currentColor;margin-right:6px;vertical-align:1px}
.dials{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:10px}
.card{background:var(--panel);border-radius:var(--r);padding:10px 12px 12px;border:2px solid transparent;transition:border-color .15s}
.card.sel{border-color:var(--acc)}
.card.linked{border-color:var(--acc)}
.ch{display:flex;align-items:center;gap:8px;font-size:13px;color:var(--dim)}
.tag{font-weight:700;color:var(--bg);background:var(--idle);border-radius:5px;padding:1px 7px}
.sel .tag,.linked .tag{background:var(--acc)}
.ch .us{margin-left:auto}
.ch .typ{font-weight:700;color:var(--acc);display:none}
.cont .ch .typ{display:inline}
svg.dial{display:block;width:100%;max-height:44vh;touch-action:none;cursor:grab;user-select:none;-webkit-user-select:none}
svg.dial.drag{cursor:grabbing}
.deg{font:700 26px/1 system-ui,sans-serif;fill:var(--text)}
.off .deg{fill:var(--off)}
.row{display:grid;grid-template-columns:repeat(5,1fr);gap:6px;margin-top:6px}
button,select,input{font:inherit;color:var(--text)}
button{background:var(--track);border:0;border-radius:10px;min-height:44px;padding:0 6px;font-weight:600;cursor:pointer;touch-action:manipulation;user-select:none;-webkit-user-select:none}
button:active{filter:brightness(1.35)}
button.on{background:var(--acc);color:var(--bg)}
button.warn{background:var(--bad);color:#fff}
.bar{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px;background:var(--panel);border-radius:var(--r);padding:10px}
.seg{display:flex;flex:1 1 220px;gap:4px;align-items:center}
.seg span{color:var(--dim);font-size:13px;margin-right:4px}
.seg button{flex:1}
.tg{display:flex;flex:1 1 260px;gap:6px}
.tg button{flex:1}
details{margin-top:10px;background:var(--panel);border-radius:var(--r);padding:0 12px}
summary{padding:12px 0;cursor:pointer;color:var(--dim);font-weight:600}
.set{display:grid;gap:10px;padding-bottom:14px}
.trim{display:flex;align-items:center;gap:10px}.trim>span{width:70px;color:var(--dim);font-size:14px}
.trim output{flex:1;text-align:center;font-weight:600}
.trim button{flex:0 0 64px}
.set h3{margin:6px 0 0;font-size:13px;color:var(--dim);font-weight:600;text-transform:uppercase;letter-spacing:.06em}
label{display:flex;align-items:center;gap:10px}
label>span{width:70px;color:var(--dim);font-size:14px}
select,input{flex:1;min-height:42px;background:var(--bg);border:1px solid var(--track);border-radius:10px;padding:0 10px}
.btns{display:flex;gap:8px}.btns button{flex:1}
.note{font-size:13px;color:var(--dim)}
kbd{background:var(--track);border-radius:4px;padding:0 5px;font:12px ui-monospace,monospace;color:var(--text)}
.keys{display:none}
@media (hover:hover) and (pointer:fine){.keys{display:block}}
@media (orientation:portrait) and (max-width:600px){svg.dial{max-height:27vh}.card{padding:8px 10px 10px}}
</style>
</head>
<body>
<main>
<header>
 <h1>SERVO CONTROL</h1>
 <span id="pwm" class="pill">PWM</span>
 <span id="conn" class="pill bad">offline</span>
</header>

<section class="dials" id="dials"></section>

<section class="bar">
 <div class="seg"><span>krok</span>
  <button data-k="0">1&deg;</button><button data-k="1">5&deg;</button><button data-k="2">15&deg;</button><button data-k="3">45&deg;</button>
 </div>
 <div class="tg">
  <button id="bLink">Link</button>
  <button id="bDemo">Demo</button>
  <button id="bCenter">90&deg; oba</button>
  <button id="bPwm">PWM</button>
 </div>
</section>

<details id="settings">
 <summary>Ustawienia</summary>
 <div class="set">
  <h3>Piny</h3>
  <label><span>Serwo 1</span><select id="pin0"></select></label>
  <label><span>Serwo 2</span><select id="pin1"></select></label>
  <div class="btns"><button id="bPins">Zapisz piny</button></div>
  <h3>Typ serwa</h3>
  <label><span>Serwo 1</span><select id="typ0"><option value="0">180&deg; &ndash; ustawia k&#261;t</option><option value="1">360&deg; &ndash; ci&#261;g&#322;e obroty (pr&#281;dko&#347;&#263;)</option></select></label>
  <div class="trim" id="trimRow0"><span>Trim S1</span><button data-tr="0" data-d="-5">&minus;5</button><output id="trim0">0 &micro;s</output><button data-tr="0" data-d="5">+5</button></div>
  <label><span>Serwo 2</span><select id="typ1"><option value="0">180&deg; &ndash; ustawia k&#261;t</option><option value="1">360&deg; &ndash; ci&#261;g&#322;e obroty (pr&#281;dko&#347;&#263;)</option></select></label>
  <div class="trim" id="trimRow1"><span>Trim S2</span><button data-tr="1" data-d="-5">&minus;5</button><output id="trim1">0 &micro;s</output><button data-tr="1" data-d="5">+5</button></div>
  <div class="note">Serwo 360&deg; nie trzyma k&#261;ta: &bdquo;90&deg;&rdquo; to STOP, odchylenie w lewo/prawo to kierunek i pr&#281;dko&#347;&#263;. Je&#347;li na STOP wolno si&#281; kr&#281;ci, popraw trim.</div>
  <h3>Sie&#263; Wi-Fi</h3>
  <div class="note" id="staInfo">&ndash;</div>
  <label><span>Tryb</span><select id="mode"><option value="0">W&#322;asna sie&#263; Cardputera (+ opcjonalnie istniej&#261;ca)</option><option value="1">Tylko istniej&#261;ca sie&#263; (bez w&#322;asnej)</option></select></label>
  <label><span>SSID</span><input id="ssid" autocomplete="off" autocapitalize="off" spellcheck="false"></label>
  <label><span>Has&#322;o</span><input id="pass" type="password" autocomplete="off"></label>
  <div class="btns"><button id="bWifi">Zapisz i po&#322;&#261;cz</button><button id="bForget" class="warn">Zapomnij</button></div>
  <div class="note">W&#322;asna sie&#263;: <b id="apName"></b>. W trybie &bdquo;tylko istniej&#261;ca sie&#263;&rdquo; Cardputer wy&#322;&#261;cza swoj&#261; sie&#263; &ndash; stron&#281; otworzysz pod adresem IP z ekranu Wi-Fi (klawisz <kbd>w</kbd>) lub http://servo.local/. Sie&#263; mo&#380;na te&#380; wybra&#263; na Cardputerze: <kbd>w</kbd>, potem <kbd>n</kbd>.</div>
 </div>
</details>

<p class="note keys">Klawiatura: <kbd>1</kbd> <kbd>2</kbd> <kbd>Tab</kbd> wyb&oacute;r serwa &middot;
<kbd>&larr;</kbd> <kbd>&rarr;</kbd> (<kbd>,</kbd> <kbd>/</kbd>) ruch &middot; <kbd>&uarr;</kbd> <kbd>&darr;</kbd> (<kbd>;</kbd> <kbd>.</kbd>) krok &middot;
<kbd>c</kbd> 90&deg; &middot; <kbd>0</kbd> <kbd>9</kbd> 0/180&deg; &middot; <kbd>l</kbd> link &middot; <kbd>d</kbd> demo &middot; <kbd>x</kbd> PWM.
Kó&#322;ko myszy nad tarcz&#261; = &plusmn;1&deg; (z <kbd>Shift</kbd> = krok).</p>
</main>

<script>
"use strict";
const STEPS=[1,5,15,45], SPEED=500; // st./s, jak symulacja na Cardputerze
const $=s=>document.querySelector(s);
let S={t:[90,90],o:1,l:0,d:0,s:0,k:1,g:[1,2],c:[0,0],r:[0,0]};  // stan z urzadzenia
const sim=[90,90], drag=[null,null], pend=[null,null];
let ws=null, online=false, pins=[];

// ---------- tarcze ----------
const NS="http://www.w3.org/2000/svg", CX=110, CY=110, R=88, VH=166;
function pt(r,deg){const a=deg*Math.PI/180;return[CX-r*Math.cos(a),CY-r*Math.sin(a)]}
function arc(r,a0,a1){const[x0,y0]=pt(r,a0),[x1,y1]=pt(r,a1);return`M${x0} ${y0}A${r} ${r} 0 0 1 ${x1} ${y1}`}
function el(tag,attrs,parent){const e=document.createElementNS(NS,tag);for(const k in attrs)e.setAttribute(k,attrs[k]);parent&&parent.appendChild(e);return e}

const D=[0,1].map(i=>{
 const card=document.createElement("div");card.className="card";
 card.innerHTML=`<div class="ch"><span class="tag">S${i+1}</span><span class="typ">360&deg;</span><span class="pin"></span><span class="us"></span></div>`;
 const svg=el("svg",{viewBox:"0 0 220 "+VH,class:"dial"},card);
 el("path",{d:arc(R,0,180),stroke:"#283042","stroke-width":16,fill:"none","stroke-linecap":"round"},svg);
 const fill=el("path",{stroke:"#ff961e","stroke-width":16,fill:"none","stroke-linecap":"round"},svg);
 for(let t=0;t<=180;t+=15){const[x0,y0]=pt(R+11,t),[x1,y1]=pt(R+(t%90?15:19),t);
  el("line",{x1:x0,y1:y0,x2:x1,y2:y1,stroke:"#7882a0","stroke-width":t%45?1:2},svg)}
 el("rect",{x:CX-38,y:CY-10,width:76,height:30,rx:6,fill:"#1e5ac8"},svg);
 const arm=el("g",{},svg);
 el("line",{x1:-16,y1:0,x2:R-22,y2:0,stroke:"#ebeef5","stroke-width":7,"stroke-linecap":"round"},arm);
 el("circle",{cx:0,cy:0,r:11,fill:"#ebeef5"},arm);el("circle",{cx:0,cy:0,r:4,fill:"#1e5ac8"},arm);
 const dot=el("circle",{r:6,fill:"#ebeef5",stroke:"#0a0e16","stroke-width":2},svg);
 const deg=el("text",{x:CX,y:VH-3,"text-anchor":"middle",class:"deg"},svg);
 const row=document.createElement("div");row.className="row";
 row.innerHTML=`<button data-a="0">0&deg;</button><button data-m="-1">&minus;</button><button data-a="90">90&deg;</button><button data-m="1">+</button><button data-a="180">180&deg;</button>`;
 card.appendChild(row);$("#dials").appendChild(card);

 // dotyk / mysz: przeciagniecie po tarczy ustawia kat (kazdy palec osobno)
 const angleAt=e=>{const b=svg.getBoundingClientRect(),sx=220/b.width,sy=VH/b.height;
  const x=(e.clientX-b.left)*sx,y=(e.clientY-b.top)*sy;
  let a=Math.atan2(CY-y,CX-x)*180/Math.PI;if(a<0)a=a<-90?180:0;return Math.round(a)};
 svg.addEventListener("pointerdown",e=>{e.preventDefault();svg.setPointerCapture(e.pointerId);
  svg.classList.add("drag");drag[i]=angleAt(e);select(i);setAngle(i,drag[i])});
 svg.addEventListener("pointermove",e=>{if(drag[i]===null||!svg.hasPointerCapture(e.pointerId))return;
  const a=angleAt(e);if(a!==drag[i]){drag[i]=a;setAngle(i,a)}});
 const end=()=>{drag[i]=null;svg.classList.remove("drag")};
 svg.addEventListener("pointerup",end);svg.addEventListener("pointercancel",end);
 svg.addEventListener("wheel",e=>{e.preventDefault();select(i);
  const d=(e.deltaY<0||e.deltaX>0?1:-1)*(e.shiftKey?STEPS[S.k]:1);setAngle(i,S.t[i]+d)},{passive:false});
 row.addEventListener("click",e=>{const b=e.target.closest("button");if(!b)return;select(i);buzz();
  if(b.dataset.a!==undefined)setAngle(i,+b.dataset.a);else setAngle(i,S.t[i]+(+b.dataset.m)*STEPS[S.k])});
 return{card,fill,arm,dot,deg,pin:card.querySelector(".pin"),us:card.querySelector(".us"),btn:row.querySelectorAll("button")};
});

// ---------- akcje ----------
function buzz(){navigator.vibrate&&navigator.vibrate(8)}
function send(m){if(ws&&ws.readyState===1)ws.send(m)}
function clamp(a){return Math.max(0,Math.min(180,Math.round(a)))}
// optymistycznie od razu na ekranie; w trybie link drugie serwo przesuwa sie o te sama roznice
function setAngle(i,a){a=clamp(a);const d=a-S.t[i];if(!d)return;
 S.t[i]=a;if(S.l)S.t[i^1]=clamp(S.t[i^1]+d);S.d=0;pend[i]=a;render()}
function setBoth(a){S.t=[a,a];S.d=0;pend[0]=pend[1]=null;send("b "+a);render()}
function select(i){if(S.s!==i){S.s=i;send("s "+i);render()}}
function toggle(key,cmd){S[key]=S[key]?0:1;send(cmd+" "+S[key]);buzz();render()}
$("#bLink").onclick=()=>toggle("l","l");
$("#bDemo").onclick=()=>toggle("d","d");
$("#bPwm").onclick=()=>toggle("o","x");
$("#bCenter").onclick=()=>{buzz();setBoth(90)};
document.querySelectorAll("[data-k]").forEach(b=>b.onclick=()=>{S.k=+b.dataset.k;send("k "+S.k);render()});

// wysylka katow max ~30x/s, zawsze ostatnia wartosc
setInterval(()=>{for(let i=0;i<2;i++)if(pend[i]!==null&&ws&&ws.readyState===1&&ws.bufferedAmount<256){
 send("a "+i+" "+pend[i]);pend[i]=null}},33);

// klawiatura (komputer) - te same klawisze co na Cardputerze
addEventListener("keydown",e=>{
 if(e.target.matches("input,select")||e.ctrlKey||e.metaKey||e.altKey)return;
 const k=e.key, i=S.s, st=STEPS[S.k];
 const mv=d=>{if(S.l){setAngle(0,S.t[0]+d)}else setAngle(i,S.t[i]+d)};
 switch(k){
  case"1":select(0);break; case"2":select(1);break;
  case"Tab":select(S.s^1);break;
  case"ArrowLeft":case",":mv(-st);break;
  case"ArrowRight":case"/":mv(st);break;
  case"ArrowUp":case";":if(S.k<3){S.k++;send("k "+S.k);render()}break;
  case"ArrowDown":case".":if(S.k>0){S.k--;send("k "+S.k);render()}break;
  case"c":$("#bCenter").click();break;
  case"0":S.l?setBoth(0):setAngle(i,0);break;
  case"9":S.l?setBoth(180):setAngle(i,180);break;
  case"l":toggle("l","l");break; case"d":toggle("d","d");break; case"x":toggle("o","x");break;
  default:return}
 e.preventDefault()});

// ---------- rysowanie ----------
const BTN180=["0°","−","90°","+","180°"], BTN360=["◀ max","−","STOP","+","max ▶"];
function usOf(i){return S.c[i]?Math.round(1500+S.r[i]+(S.t[i]-90)*500/90):Math.round(500+1900*S.t[i]/180)}
function speed(t){const p=Math.trunc((t-90)*100/90);return p?(p>0?"+":"")+p+"%":"STOP"}
function render(){
 for(let i=0;i<2;i++){const d=D[i],act=S.l||S.s===i,on=!!S.o,c=!!S.c[i];
  d.card.classList.toggle("sel",act);d.card.classList.toggle("off",!on);d.card.classList.toggle("cont",c);
  d.btn.forEach((b,k)=>b.textContent=(c?BTN360:BTN180)[k]);
  const ty=$("#typ"+i);if(document.activeElement!==ty)ty.value=c?1:0;
  $("#trimRow"+i).style.display=c?"":"none";$("#trim"+i).textContent=(S.r[i]>0?"+":"")+S.r[i]+" µs";
  d.fill.setAttribute("stroke",on?(act?"#ff961e":"#5aa0e6"):"#5a5a64");
  const g=pins.find(p=>p[0]===S.g[i]);
  d.pin.textContent="G"+S.g[i]+(g?" · "+g[1]:"");
  d.us.textContent=(on?usOf(i):0)+" µs"}
 $("#bLink").classList.toggle("on",!!S.l);$("#bDemo").classList.toggle("on",!!S.d);
 $("#bPwm").classList.toggle("on",!!S.o);$("#bPwm").textContent=S.o?"PWM ON":"PWM OFF";
 const p=$("#pwm");p.textContent=S.o?"PWM ON":"PWM OFF";p.className="pill "+(S.o?"ok":"bad");
 document.querySelectorAll("[data-k]").forEach(b=>b.classList.toggle("on",+b.dataset.k===S.k));
}
let last=performance.now();
function frame(now){const dt=Math.min(.1,(now-last)/1000);last=now;
 for(let i=0;i<2;i++){const d=D[i];
  const c=!!S.c[i];
  if(c)sim[i]=S.t[i]; // serwo 360: wskazowka predkosci bez opoznienia
  else if(S.o){const diff=S.t[i]-sim[i],st=SPEED*dt;sim[i]=Math.abs(diff)<=st?S.t[i]:sim[i]+Math.sign(diff)*st}
  d.fill.setAttribute("d",c?(Math.abs(sim[i]-90)>.5?arc(R,Math.min(90,sim[i]),Math.max(90,sim[i])):""):(sim[i]>.5?arc(R,0,sim[i]):""));
  d.arm.setAttribute("transform",`translate(${CX} ${CY}) rotate(${sim[i]-180})`);
  const[x,y]=pt(R,S.t[i]);d.dot.setAttribute("cx",x);d.dot.setAttribute("cy",y);
  d.dot.style.display=Math.abs(S.t[i]-sim[i])>.5?"":"none";
  d.deg.textContent=c?speed(S.t[i]):Math.round(sim[i])+"°"}
 requestAnimationFrame(frame)}
requestAnimationFrame(frame);

// ---------- polaczenie ----------
function connect(){
 ws=new WebSocket(`ws://${location.hostname||"192.168.4.1"}:81/`);
 ws.onopen=()=>{online=true;$("#conn").textContent="online";$("#conn").className="pill ok"};
 ws.onclose=()=>{online=false;$("#conn").textContent="offline";$("#conn").className="pill bad";setTimeout(connect,1000)};
 ws.onerror=()=>ws.close();
 ws.onmessage=e=>{let m;try{m=JSON.parse(e.data)}catch(_){return}
  for(let i=0;i<2;i++)if(drag[i]!==null||pend[i]!==null)m.t[i]=S.t[i]; // nie skacz pod palcem
  const pinsChanged=!S.g||m.g[0]!==S.g[0]||m.g[1]!==S.g[1];
  if(!m.c)m.c=[0,0];if(!m.r)m.r=[0,0];
  S=m;render();if(pinsChanged)fillPins()}}
// strona wraca z tla na telefonie -> natychmiast ponow polaczenie
document.addEventListener("visibilitychange",()=>{if(!document.hidden&&ws&&ws.readyState>1)connect()});

// ---------- ustawienia ----------
function fillPins(){for(let i=0;i<2;i++){const s=$("#pin"+i);
 s.innerHTML=pins.map(p=>`<option value="${p[0]}">G${p[0]} – ${p[1]}${p[2]?" (linia SD)":""}</option>`).join("");
 s.value=S.g[i]}}
$("#bPins").onclick=()=>{const a=+$("#pin0").value,b=+$("#pin1").value;
 if(a===b){alert("Serwa muszą mieć różne piny");return}send("p "+a+" "+b);buzz()};
async function info(){try{const j=await(await fetch("/info")).json();
 if(!pins.length){pins=j.pins;fillPins();render()}
 $("#apName").textContent=j.ap;
 if(document.activeElement!==$("#mode"))$("#mode").value=j.mode||0;
 const st={0:"nie skonfigurowana",1:"łączenie z "+j.ssid+"…",2:"połączono z "+j.ssid+" – http://"+j.ip+"/ (lub http://servo.local/)",3:"nie udało się połączyć z "+j.ssid}[j.sta];
 $("#staInfo").textContent="Tryb: "+(j.mode?"tylko istniejąca sieć":"własna sieć")+" · status: "+st;
 if(!$("#ssid").value&&j.ssid&&document.activeElement!==$("#ssid"))$("#ssid").value=j.ssid}catch(_){}}
async function postWifi(ssid,pass,mode){const f=new URLSearchParams({ssid,pass,mode});
 try{await fetch("/wifi",{method:"POST",body:f})}catch(_){}setTimeout(info,500)}
$("#bWifi").onclick=()=>{const s=$("#ssid").value.trim();if(!s){$("#ssid").focus();return}
 const m=$("#mode").value;
 if(m==="1"&&!confirm("Cardputer wyłączy swoją sieć i połączy się z „"+s+"”. Stronę otworzysz potem pod adresem IP z ekranu Wi-Fi lub http://servo.local/ (będąc w tej samej sieci). Kontynuować?"))return;
 postWifi(s,$("#pass").value,m)};
$("#bForget").onclick=()=>{$("#ssid").value="";$("#pass").value="";postWifi("","","0")};
[0,1].forEach(i=>$("#typ"+i).onchange=e=>{S.c[i]=+e.target.value;S.t[i]=90;send("m "+i+" "+S.c[i]);render()});
document.querySelectorAll("[data-tr]").forEach(b=>b.onclick=e=>{e.preventDefault();const i=+b.dataset.tr;
 S.r[i]=Math.max(-200,Math.min(200,S.r[i]+(+b.dataset.d)));send("r "+i+" "+S.r[i]);buzz();render()});
setInterval(()=>{if($("#settings").open)info()},2000);

render();info();connect();
</script>
</body>
</html>)HTML";
