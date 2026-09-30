// Node's built-in runner: no browser packages or flight simulator required.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../web/viewer.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
// Compile the entire page script, then exercise its asynchronous data layer
// with controllable HTTP responses and DOM elements.
new vm.Script(script);
const dataLayer = script.slice(script.indexOf('let libraryMode'), script.indexOf('function bindCamera'));

function page() {
  const elements = new Map();
  const pending = [];
  const context = vm.createContext({
    console,
    fetch(url) { return new Promise(resolve => pending.push({url, resolve})); },
    clearTimeout() {}, setTimeout() { return 1; },
    document: {
      getElementById(id) {
        if (!elements.has(id)) elements.set(id, {textContent:'', value:'', innerHTML:'',
          options:[], appendChild(o){this.options.push(o);}, classList:{add(){},remove(){}}});
        return elements.get(id);
      },
      createElement() { return {}; },
    },
    refreshRange(){}, setPlay(){},
  });
  vm.runInContext(`let mode='live',liveTimer=null,goal=null,mapSize=[40,40,10];
    let states=[],points=[],plans=[],maps=[],geometry=[];
    let fullPersistent=[],fullPersistentUnknown=[],persistent=[],persistentUnknown=[];
    let rangeBbox=null,curT=0,t0=0,tEnd=0,playing=false;`, context);
  vm.runInContext(dataLayer, context);
  function reply(index, body, ok=true) {pending[index].resolve({ok,status:ok?200:422,json:async()=>body});}
  function read(expr) {return vm.runInContext(expr,context);}
  return {context,pending,reply,read,elements};
}

test('late recording response cannot overwrite a newer selection', async () => {
  const p=page();
  const older=p.read("loadRecordingById('old')");
  const newer=p.read("loadRecordingById('new')");
  p.reply(1,{goal:[2,0,0],states:[]}); await newer;
  p.reply(0,{goal:[1,0,0],states:[]}); await older;
  assert.equal(p.read('goal[0]'),2);
});

test('switching to live ignores pending replay and clears historical map', async () => {
  const p=page(); const older=p.read("loadRecordingById('old')");
  p.read('fullPersistent=[1,2,3]; persistent=[1,2,3]; startLive()');
  p.reply(0,{goal:[9,0,0],states:[]}); await older;
  assert.equal(p.read('mode'),'live');
  assert.equal(p.read('persistent.length'),0);
  assert.equal(p.read('goal'),null);
});

test('duplicate live frames are ignored but empty map updates survive', () => {
  const p=page();
  p.read("appendLive({state:{seq:1,t:1},map:{seq:2,t:1,occ:[1,2,3]}})");
  p.read("appendLive({state:{seq:1,t:1},map:{seq:2,t:1,occ:[1,2,3]}})");
  p.read("appendLive({state:{seq:3,t:2},map:{seq:4,t:2,occ:[]}})");
  assert.equal(p.read('states.length'),2);
  assert.equal(p.read('maps.length'),2);
  assert.equal(p.read('maps[1].occ.length'),0);
});

test('HTTP errors are reported without applying an empty success response', async () => {
  const p=page(); const loaded=p.read("loadRecordingById('bad')");
  p.reply(0,{error:'checksum mismatch'},false); await loaded;
  assert.match(p.elements.get('status').textContent,/checksum mismatch/);
});

test('refresh preserves selection and incomplete recordings remain readable', async () => {
  const p=page();
  p.context.document.getElementById('selRec').value='chosen';
  const loaded=p.read('refreshRecordings()');
  p.reply(0,[{id:'new',name:'new',n_states:1},{id:'chosen',name:'chosen',n_states:1}]);
  // Drain the async fetch/JSON continuations until the recording request starts.
  for(let n=0;n<8;n++) await Promise.resolve();
  assert.match(p.pending[1].url,/id=chosen/);
  p.reply(1,{complete:false,states:[]}); await loaded;
  assert.match(p.elements.get('status').textContent,/未完整结束/);
});
