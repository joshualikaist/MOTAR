// Local browser presentation check. Chrome uses software rendering; no simulator/policy imports.
// Usage: node <this file> <new output directory>
'use strict';
const fs = require('fs'), path = require('path'), http = require('http');
const cp = require('child_process'), os = require('os');
const root = path.resolve(__dirname, '../../..');
const out = path.resolve(process.argv[2]);
fs.mkdirSync(out, {recursive: false});
const profile = fs.mkdtempSync(path.join(os.tmpdir(), 'motar-docs-browser-'));
const baseline = '2da5b12c1954ee768fcba59d1fb1fb9c78f7d536';
const mime = {'.html':'text/html', '.css':'text/css', '.js':'application/javascript',
  '.json':'application/json', '.svg':'image/svg+xml', '.png':'image/png'};
const server = http.createServer((req, res) => {
  try {
    const url = new URL(req.url, 'http://localhost');
    let name = decodeURIComponent(url.pathname);
    const old = name.indexOf('/before/') === 0;
    name = name.replace(/^\/(before|after)\//, '');
    if (name.endsWith('/')) name += 'index.html';
    const full = path.resolve(root, name);
    if (!full.startsWith(root + path.sep)) throw Error('outside root');
    const data = old ? cp.execFileSync('git', ['show', baseline+':'+name],
      {cwd:root, maxBuffer:30*1024*1024, stdio:['ignore','pipe','ignore']}) : fs.readFileSync(full);
    res.writeHead(200, {'Content-Type':mime[path.extname(name)] || 'application/octet-stream'});
    res.end(data);
  } catch(e) {res.writeHead(404); res.end('Not found');}
});
const sleep = ms => new Promise(resolve => setTimeout(resolve,ms));
let chrome, seq = 0, session, waiting = new Map();
function call(method, params, browser) {
  return new Promise((resolve,reject) => {
    const id = ++seq;
    const timer = setTimeout(() => {waiting.delete(id); reject(Error('CDP timeout: '+method));}, 20000);
    waiting.set(id, {resolve,reject,timer});
    const packet = {id,method,params:params || {}};
    if (!browser) packet.sessionId = session;
    chrome.stdio[3].write(JSON.stringify(packet)+'\0');
  });
}
async function js(expression) {
  const r = await call('Runtime.evaluate',{expression,returnByValue:true,awaitPromise:true});
  if (r.exceptionDetails) throw Error(JSON.stringify(r.exceptionDetails));
  return r.result.value;
}
(async () => {
  await new Promise(resolve => server.listen(0,'127.0.0.1',resolve));
  // The simulator shell's conda libffi breaks system Chrome; isolate this subprocess only.
  const browserEnv = Object.assign({},process.env);
  delete browserEnv.LD_LIBRARY_PATH; delete browserEnv.LD_PRELOAD;
  chrome = cp.spawn('/usr/bin/google-chrome', ['--headless=new','--no-sandbox',
    '--disable-dev-shm-usage','--disable-gpu','--use-gl=angle','--use-angle=swiftshader',
    '--enable-webgl','--enable-unsafe-swiftshader','--disable-background-networking',
    '--disable-extensions','--disable-sync','--no-first-run','--no-default-browser-check',
    '--remote-debugging-pipe','--user-data-dir='+profile,'about:blank'],
    {stdio:['ignore','ignore','ignore','pipe','pipe'],env:browserEnv});
  function pipeError(e) {for (const w of waiting.values()) {clearTimeout(w.timer);w.reject(e);} waiting.clear();}
  chrome.stdio[3].on('error',pipeError); chrome.stdio[4].on('error',pipeError);
  let buffer = '';
  chrome.stdio[4].on('data', data => {
    buffer += data.toString();
    let end;
    while ((end = buffer.indexOf('\0')) >= 0) {
      const part = buffer.slice(0,end); buffer = buffer.slice(end+1);
      if (!part) continue;
      const m = JSON.parse(part), w = waiting.get(m.id);
      if (w) {waiting.delete(m.id); clearTimeout(w.timer);
        if (m.error) w.reject(Error(JSON.stringify(m.error))); else w.resolve(m.result);}
    }
  });
  const target = await call('Target.createTarget',{url:'about:blank'},true);
  session = (await call('Target.attachToTarget',{targetId:target.targetId,flatten:true},true)).sessionId;
  await call('Page.enable');
  const rows = [];
  for (const version of ['before','after']) {
    for (const viewport of [['desktop',1360,1000],['tablet',800,1000],['mobile',390,844]]) {
      const [label,width,height] = viewport;
      await call('Emulation.setDeviceMetricsOverride',{width,height,deviceScaleFactor:1,mobile:label==='mobile'});
      for (const page of ['index.html','evidence.html']) {
        const url = 'http://127.0.0.1:'+server.address().port+'/'+version+'/docs/status/'+page;
        await call('Page.navigate',{url});
        let ready = false;
        for (let i=0;i<100;i++) {
          await sleep(100);
          ready = await js('document.readyState === "complete" && location.href === '+JSON.stringify(url));
          if (ready) break;
        }
        if (!ready) throw Error('Readiness timeout');
        const decoded = await js('Promise.all([...document.images].map(i=>{i.loading="eager";return i.decode().then(()=>true,()=>false)})).then(x=>x.every(Boolean))');
        if (page === 'index.html') {
          await js('document.querySelector("#btn-play").click()');
          await sleep(200);
        }
        const check = await js('({width:innerWidth, scrollWidth:document.documentElement.scrollWidth, h1:document.querySelectorAll("h1").length, canvas:!!document.querySelector("#stage canvas"), boia:!!document.querySelector("#boia"), flow:document.querySelectorAll(".boia-flow li").length, milestones:document.querySelectorAll("[data-boia-milestone]").length, nav:[...document.querySelectorAll("header nav a")].filter(a=>a.getClientRects().length).length, brokenImages:[...document.images].filter(i=>!i.complete||!i.naturalWidth).length})');
        Object.assign(check,{version,viewport:label,page,imagesDecoded:decoded});
        if (check.scrollWidth>width || check.h1!==1 || !decoded || check.brokenImages) throw Error(JSON.stringify(check));
        if (version==='after' && (!check.boia || check.milestones!==5 || (page==='index.html' && check.flow!==5))) throw Error(JSON.stringify(check));
        if (page==='index.html' && (!check.canvas || check.nav!==6)) throw Error('Viewer/nav check: '+JSON.stringify(check));
        rows.push(check);
        const prefix = version+'-'+label+'-'+page.replace('.html','');
        await js('document.documentElement.style.scrollBehavior="auto"; scrollTo(0,0)');
        let shot = await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:false});
        fs.writeFileSync(path.join(out,prefix+'-top.png'), Buffer.from(shot.data,'base64'));
        if (version==='after') {
          const rect = await js('(()=>{const r=document.querySelector("#boia").getBoundingClientRect();return {x:r.x+scrollX,y:r.y+scrollY,width:r.width,height:r.height,scale:1}})()');
          shot = await call('Page.captureScreenshot',{format:'png',captureBeyondViewport:true,clip:rect});
          fs.writeFileSync(path.join(out,prefix+'-boia.png'),Buffer.from(shot.data,'base64'));
        }
      }
    }
  }
  const receipt = {status:'PASS',scope:'local browser software rendering, no simulator',baseline,checks:rows};
  fs.writeFileSync(path.join(out,'preview.json'),JSON.stringify(receipt,null,2)+'\n');
  console.log(JSON.stringify(receipt));
})().catch(e => {console.error(e.stack);process.exitCode=1;}).then(async () => {
  if (chrome) {try {await call('Browser.close',{},true);}catch(e){} chrome.kill();}
  server.close();
  // Leave the disposable profile in /tmp; never delete user data or experiment output.
});
