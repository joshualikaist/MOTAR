// CPU-only browser QA. Never imports simulator, policy or research modules.
'use strict';
const fs=require('fs'),path=require('path'),http=require('http'),cp=require('child_process'),os=require('os');
const root=path.resolve(__dirname,'../../..'),out=path.resolve(process.argv[2]);
fs.mkdirSync(out,{recursive:false});
const mime={'.html':'text/html','.css':'text/css','.js':'text/javascript','.svg':'image/svg+xml','.png':'image/png','.json':'application/json'};
const server=http.createServer((req,res)=>{
 try{let n=decodeURIComponent(new URL(req.url,'http://localhost').pathname);if(n.endsWith('/'))n+='index.html';
 const full=path.resolve(root,'.'+n);if(!full.startsWith(root+path.sep))throw Error('Outside root');
 res.writeHead(200,{'Content-Type':mime[path.extname(n)]||'application/octet-stream'});res.end(fs.readFileSync(full));
 }catch(e){res.writeHead(404);res.end('Not found');}
});
let chrome,session,id=0,waiting=new Map();
const sleep=ms=>new Promise(r=>setTimeout(r,ms));
function call(method,params={},browser=false){return new Promise((resolve,reject)=>{
 const n=++id,t=setTimeout(()=>{waiting.delete(n);reject(Error('Timeout '+method));},20000);
 waiting.set(n,{resolve,reject,t});let m={id:n,method,params};if(!browser)m.sessionId=session;
 chrome.stdio[3].write(JSON.stringify(m)+'\0');
});}
async function js(expression){let r=await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});if(r.exceptionDetails)throw Error(JSON.stringify(r.exceptionDetails));return r.result.value;}
async function navigate(url){await call('Page.navigate',{url});for(let i=0;i<100;i++){await sleep(100);if(await js('document.readyState==="complete" && location.href==='+JSON.stringify(url)))return;}throw Error('Readiness');}
async function shot(name,clip){const r=await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:!!clip,...(clip?{clip}:{})});fs.writeFileSync(path.join(out,name+'.png'),Buffer.from(r.data,'base64'));}
(async()=>{
 await new Promise(r=>server.listen(0,'127.0.0.1',r));
 const env={...process.env};delete env.LD_LIBRARY_PATH;delete env.LD_PRELOAD;
 chrome=cp.spawn('/usr/bin/google-chrome',['--headless=new','--no-sandbox','--disable-dev-shm-usage','--disable-gpu','--use-gl=angle','--use-angle=swiftshader','--enable-webgl','--enable-unsafe-swiftshader','--disable-extensions','--no-first-run','--remote-debugging-pipe','--user-data-dir='+fs.mkdtempSync(path.join(os.tmpdir(),'motar-redesign-browser-')),'about:blank'],{stdio:['ignore','ignore','ignore','pipe','pipe'],env});
 let buffer='';chrome.stdio[4].on('data',d=>{buffer+=d.toString();let k;while((k=buffer.indexOf('\0'))>=0){const s=buffer.slice(0,k);buffer=buffer.slice(k+1);if(!s)continue;const m=JSON.parse(s),w=waiting.get(m.id);if(w){waiting.delete(m.id);clearTimeout(w.t);if(m.error)w.reject(Error(JSON.stringify(m.error)));else w.resolve(m.result);}}});
 const t=await call('Target.createTarget',{url:'about:blank'},true);session=(await call('Target.attachToTarget',{targetId:t.targetId,flatten:true},true)).sessionId;await call('Page.enable');
 const rows=[],base='http://127.0.0.1:'+server.address().port;
 for(const [label,width,height] of [['desktop',1360,1000],['tablet',800,1000],['mobile',390,844]]){
  await call('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:label==='mobile'});
  for(const page of ['index.html','evidence.html','figures.html']){
   await navigate(base+'/docs/status/'+page);await js('Promise.all([...document.images].map(i=>{i.loading="eager";return i.decode()}))');
   if(page==='index.html'){await js('document.querySelector("#btn-play").click()');await sleep(100);}
   const row=await js('({width:innerWidth,scrollWidth:document.documentElement.scrollWidth,h1:document.querySelectorAll("h1").length,images:document.images.length,broken:[...document.images].filter(i=>!i.complete||!i.naturalWidth).length,canvas:!!document.querySelector("#stage canvas"),nav:document.querySelectorAll("header.masthead nav a").length,figures:document.querySelectorAll(".figure-surface").length,statusUnavailable:document.body.innerText.includes("Status unavailable")})');
   Object.assign(row,{page,viewport:label});rows.push(row);
   if(row.scrollWidth>width||row.h1!==1||row.broken||row.nav!==6||row.statusUnavailable)throw Error(JSON.stringify(row));
   if(page==='index.html'&&(!row.canvas||row.figures!==6))throw Error(JSON.stringify(row));
   await shot(label+'-'+page.replace('.html','')+'-top');
   if(page==='index.html'&&label!=='tablet'){
    const rect=await js('(()=>{const r=document.querySelector("#representation").getBoundingClientRect();return{x:r.x+scrollX,y:r.y+scrollY,width:r.width,height:r.height,scale:1}})()');await shot(label+'-representation',rect);
   }
  }
 }
 const svg=[];await call('Emulation.setDeviceMetricsOverride',{width:1200,height:900,deviceScaleFactor:1,mobile:false});
 for(const f of JSON.parse(fs.readFileSync(path.join(root,'docs/assets/paper/redesign-2026-10-22/manifest.json'))).figures){
  await navigate(base+'/docs/assets/paper/redesign-2026-10-22/'+f.id+'.svg');await js('document.fonts.ready');
  const result=await js('(()=>{const s=document.querySelector("svg"),r=s.getBoundingClientRect();const ts=[...s.querySelectorAll("text")].filter(t=>t.textContent.trim()).map(t=>({text:t.textContent,r:t.getBoundingClientRect()}));let clip=[],overlap=[];for(const t of ts){if(t.r.left<r.left-2||t.r.top<r.top-2||t.r.right>r.right+2||t.r.bottom>r.bottom+2)clip.push(t.text)}for(let i=0;i<ts.length;i++)for(let j=i+1;j<ts.length;j++){const a=ts[i].r,b=ts[j].r;if(Math.min(a.right,b.right)-Math.max(a.left,b.left)>2&&Math.min(a.bottom,b.bottom)-Math.max(a.top,b.top)>2)overlap.push([ts[i].text,ts[j].text]);}return{texts:ts.length,clipped:clip,overlaps:overlap};})()');
  svg.push({figure:f.id,...result});if(result.clipped.length||result.overlaps.length)throw Error(JSON.stringify(svg[svg.length-1]));
 }
 const receipt={status:'PASS',scope:'CPU software browser; no simulator or policy',pages:rows,svg_text_geometry:svg};fs.writeFileSync(path.join(out,'preview.json'),JSON.stringify(receipt,null,2)+'\n');console.log(JSON.stringify(receipt));
})().catch(e=>{console.error(e.stack);process.exitCode=1;}).then(async()=>{if(chrome){try{await call('Browser.close',{},true);}catch{}chrome.kill();}server.close();});
