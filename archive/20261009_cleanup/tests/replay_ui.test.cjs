// Node's built-in runner: no browser packages or flight simulator required.
const {test} = require('node:test');
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const html = fs.readFileSync(path.join(__dirname, '../../../experiments/web/viewer.html'), 'utf8');
const script = html.match(/<script>([\s\S]*?)<\/script>/)[1];
// Compile the entire page script, then exercise its asynchronous data layer
// with controllable HTTP responses and DOM elements.
new vm.Script(script);

test('interpolation preserves held commands while interpolating actual motion', () => {
  const context=vm.createContext({states:[
    {t:1,p:[0,0,0],v:[2,0,0],a:[0,0,0],yaw:0,u:[3,4,0],reference_speed:8,speed_limit:6,approach_goal:[10,0,0]},
    {t:2,p:[2,0,0],v:[4,0,0],a:[0,0,0],yaw:0,u:[0,0,0],reference_speed:8,speed_limit:0,approach_goal:[20,0,0]},
  ]});
  vm.runInContext(script.slice(script.indexOf('function lerpState('),script.indexOf('function latestIdx(')),context);
  const state=vm.runInContext('lerpState(1.5)',context);
  assert.equal(state.v[0],3);
  assert.equal(Math.hypot(...state.u),5);
  assert.equal(state.reference_speed,8);
  assert.equal(state.speed_limit,6);
  assert.equal(state.approach_goal[0],10);
});

test('HUD distinguishes zero command from measured motion and absent legacy fields', () => {
  const elements=new Map();
  const context=vm.createContext({curT:1,t0:0,tEnd:2,timelineClock:'墙钟时间',goal:[10,0,0],
    current:{p:[0,0,0],v:[2,0,0],a:[0,0,0],u:[0,0,0],reference_speed:8,speed_limit:0},
    document:{getElementById(id){if(!elements.has(id))elements.set(id,{textContent:''});return elements.get(id);}},
  });
  vm.runInContext('function lerpState(){return current;}',context);
  vm.runInContext(script.slice(script.indexOf('function updateHUD('),script.indexOf('const scrub =')),context);
  vm.runInContext('updateHUD()',context);
  assert.equal(elements.get('hReference').textContent,'8.00 m/s');
  assert.equal(elements.get('hCommand').textContent,'0.00 m/s');
  assert.equal(elements.get('hVmag').textContent,'2.00 m/s');
  assert.equal(elements.get('hLimit').textContent,'0.00 m/s');
  vm.runInContext('delete current.u;delete current.reference_speed;delete current.speed_limit;updateHUD()',context);
  assert.equal(elements.get('hCommand').textContent,'-');
  assert.equal(elements.get('hReference').textContent,'-');
});

test('simulation replay maps sensor and plan times with state anchors without changing recorded data', () => {
  const p=page();
  p.read('input={states:[{t:100,sim_timestamp_ns:1000000000},{t:102,sim_timestamp_ns:2000000000},{t:104,sim_timestamp_ns:4000000000}],points:[{t:101}],plans:[{t:103}],maps:[{t:99}]};output=simulationClockRecording(input)');
  assert.equal(p.read('output.states[2].t-output.states[0].t'),3);
  assert.equal(p.read('output.points[0].t'),100.5);
  assert.equal(p.read('output.plans[0].t'),102);
  assert.equal(p.read('output.maps[0].t'),99.5);
  assert.equal(p.read('input.states[2].t'),104);
  assert.equal(p.read('input.points[0].t'),101);
});

test('legacy recordings and simulation resets retain the wall clock', () => {
  const p=page();
  assert.equal(p.read('input={states:[{t:1},{t:2}]};simulationClockRecording(input)===input'),true);
  assert.equal(p.read('input={states:[{t:1,sim_timestamp_ns:20},{t:2,sim_timestamp_ns:10}]};simulationClockRecording(input)===input'),true);
});

test('target timestamps use the same simulation clock as the pursuer', () => {
  const p=page();
  p.read('input={states:[{t:100,sim_timestamp_ns:1000000000},{t:104,sim_timestamp_ns:3000000000}],targets:[{t:102,sim_timestamp_ns:2000000000,p:[4,0,0]}]};output=simulationClockRecording(input)');
  assert.equal(p.read('output.targets[0].t'),101);
  assert.equal(p.read('input.targets[0].t'),102);
});

test('target replay is cleared when switching recordings and live mode', () => {
  const p=page();
  p.read('applyRecording({states:[],targets:[{t:1,p:[4,0,0]}]})');
  assert.equal(p.read('targets.length'),1);
  p.read('applyRecording({states:[]})');
  assert.equal(p.read('targets.length'),0);
  p.read('appendLive({target:{seq:1,t:1,p:[4,0,0]}});appendLive({target:{seq:1,t:1,p:[4,0,0]}})');
  assert.equal(p.read('targets.length'),1);
  p.read('startLive()');
  assert.equal(p.read('targets.length'),0);
});
const dataLayer = script.slice(script.indexOf('let libraryMode'), script.indexOf('function bindCamera'));

function page() {
  const elements = new Map();
  const pending = [];
  const context = vm.createContext({
    console,
    fetch(url, options) { return new Promise(resolve => pending.push({url, options, resolve})); },
    clearTimeout() {}, setTimeout() { return 1; },
    document: {
      getElementById(id) {
        if (!elements.has(id)) elements.set(id, {textContent:'', value:'', innerHTML:'',style:{},
          options:[], appendChild(o){this.options.push(o);}, classList:{add(){},remove(){}},
          listeners:{}, addEventListener(type,fn){this.listeners[type]=fn;},focus(){},setPointerCapture(){}});
        return elements.get(id);
      },
      createElement() { return {}; },
    },
    refreshRange(){}, setPlay(){},
  });
  vm.runInContext(`let mode='live',liveTimer=null,goal=null,mapSize=[40,40,10];
    let states=[],targets=[],points=[],plans=[],maps=[],geometry=[];
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

test('example collection filters runs and sorts cases by catalog number', async () => {
  const p=page();
  p.context.document.getElementById('selCollection').value='examples';
  const loaded=p.read('refreshRecordings(false)');
  p.reply(0,[{id:'run',collection:'results',name:'new'},
    {id:'second',collection:'examples',example_order:2,display_name:'02 复飞'},
    {id:'first',collection:'examples',example_order:1,display_name:'01 首飞'}]);
  await loaded;
  assert.deepEqual(p.elements.get('selRec').options.map(o=>o.value),['first','second']);
  assert.match(p.elements.get('selRec').options[0].textContent,/01 首飞/);
  assert.equal(p.elements.get('selCollection').value,'examples');
});

test('late library scan cannot override the newer collection selection', async () => {
  const p=page();
  const old=p.read('refreshRecordings(false)');
  p.context.document.getElementById('selCollection').value='examples';
  const current=p.read('refreshRecordings(false)');
  p.reply(1,[{id:'example',collection:'examples',name:'example'}]); await current;
  p.reply(0,[{id:'run',collection:'results',name:'run'}]); await old;
  assert.deepEqual(p.elements.get('selRec').options.map(o=>o.value),['example']);
});

test('empty collection clears the previous flight and historical map', async () => {
  const p=page();
  p.read('goal=[1,2,3];states=[{t:1,p:[0,0,0]}];fullPersistent=[1,2,3]');
  const loaded=p.read('refreshRecordings()');p.reply(0,[]);await loaded;
  assert.equal(p.read('goal'),null);
  assert.equal(p.read('states.length'),0);
  assert.equal(p.read('fullPersistent.length'),0);
  assert.match(p.elements.get('status').textContent,/当前栏目没有录制/);
});

test('scene synchronization sends relative time and rejects a late frame from another recording', async () => {
  const p=page();
  p.read("simulatorAvailable=true;simulatorEnabled=true;mode='replay';simulatorRecordingId='first';states=[{t:100}];t0=100;curT=102;");
  const operation=p.read('syncSimulator(1000)');
  assert.equal(p.pending[0].url,'/api/simulator');
  assert.equal(JSON.parse(p.pending[0].options.body).seconds,2);
  p.read("simulatorRecordingId='second'");
  p.reply(0,{frame_available:true,recording_id:'first',frame_sequence:1});await operation;
  assert.equal(p.read('simulatorLastImage'),0);
});

test('scene mouse gestures orbit, pan and zoom without changing playback time', async () => {
  const p=page();
  p.read("simulatorAvailable=true;simulatorEnabled=true;mode='replay';simulatorRecordingId='case';states=[{t:100}];t0=100;curT=102;");
  vm.runInContext(script.slice(script.indexOf('function bindSimulatorCamera('),script.indexOf('/* ============================ 交互')),p.context);
  p.read('bindSimulatorCamera()');
  const image=p.context.document.getElementById('simImage');
  const event=(fields={})=>({pointerId:1,button:0,clientX:0,clientY:0,preventDefault(){},...fields});
  image.listeners.pointerdown(event());
  image.listeners.pointermove(event({clientX:100,clientY:30}));
  image.listeners.pointerup(event());
  assert.ok(Math.abs(p.read('simulatorCamera.azimuth'))>.5);
  assert.ok(p.read('simulatorCamera.elevation')>0);
  image.listeners.pointerdown(event({button:2}));
  image.listeners.pointermove(event({clientX:40,clientY:20}));
  image.listeners.pointercancel(event());
  assert.equal(p.read('simulatorCamera.pan[0]'),-.06);
  image.listeners.wheel(event({deltaY:-200,deltaMode:0}));
  assert.ok(p.read('simulatorCamera.zoom')<1);
  assert.equal(p.read('curT'),102);
  const operation=p.read('syncSimulator(1000)');
  const payload=JSON.parse(p.pending[0].options.body);
  assert.ok(payload.camera.zoom<1 && payload.camera.azimuth<0 && payload.camera.pan[0]<0);
  p.reply(0,{});await operation;
  image.listeners.dblclick(event());
  assert.equal(p.read('simulatorCamera.zoom'),1);
  assert.equal(p.read('simulatorCamera.azimuth'),0);
  assert.equal(p.read('simulatorCamera.pan[0]'),0);
});

test('scene camera limits allow reversing immediately and resetting on a new recording', () => {
  const p=page();
  p.read("document.getElementById('simView').value='top';moveSimulatorCamera(1e5,1e5)");
  const high=p.read('simulatorCamera.elevation');
  p.read('moveSimulatorCamera(0,-10)');
  assert.ok(p.read('simulatorCamera.elevation')<high);
  assert.ok(Math.abs(p.read('simulatorCamera.azimuth'))<=Math.PI);
  p.read('zoomSimulatorCamera(-1e6);zoomSimulatorCamera(-1e6);moveSimulatorCamera(1e6,1e6,true)');
  assert.equal(p.read('simulatorCamera.zoom'),.15);
  assert.equal(p.read('simulatorCamera.pan[0]'),-10);
  p.read('beginReplay()');
  assert.equal(p.read('simulatorCamera.zoom'),1);
  assert.equal(p.read('simulatorCamera.elevation'),0);
});

test('unified replay enables scene selection while respecting the initial view preference', async () => {
  for(const [available,selected] of [[true,false],[true,true],[false,false]]){
    const p=page(),loading=p.read('loadStatus()');
    p.reply(0,{simulator:available,simulator_default:selected});await loading;
    assert.equal(p.context.document.getElementById('simEnabled').disabled,!available);
    assert.equal(p.context.document.getElementById('simEnabled').checked,selected);
  }
});
