"""Versioned append-only recording with bounded live buffers and atomic commits."""
import gzip
import copy
import hashlib
import json
import os
import queue
import threading
import time
from collections import deque,OrderedDict
from pathlib import Path
import numpy as np

STREAMS=('states','points','plans','maps')

def _timestamp(value):
    try:
        value = float(value)
    except (ValueError, TypeError, OverflowError):
        return None
    return value if np.isfinite(value) else None

def _array(value):
    try:
        a=np.asarray(value,dtype=float).reshape(-1)
        a=np.nan_to_num(a,nan=0.,posinf=0.,neginf=0.)
        return a[:len(a)//3*3].reshape(-1,3).copy()
    except (ValueError,TypeError): return np.empty((0,3))

def _vec(value):
    a=_array(value)
    return a[0].copy() if len(a) else np.zeros(3)

def _jsonable(value):
    if isinstance(value,np.ndarray): return np.round(value,5).tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,dict): return {k:_jsonable(v) for k,v in value.items()}
    if isinstance(value,(list,tuple,deque)): return [_jsonable(v) for v in value]
    return value

def atomic_json(path,data):
    path=Path(path); path.parent.mkdir(parents=True,exist_ok=True)
    tmp=path.with_name(path.name+'.tmp')
    with tmp.open('w',encoding='utf-8') as f:
        json.dump(data,f,ensure_ascii=False,allow_nan=False,separators=(',',':')); f.flush(); os.fsync(f.fileno())
    os.replace(tmp,path)

def _meta(data):
    st=data.get('states',[])
    return dict(goal=data.get('goal'),t0=st[0]['t'] if st else None,t1=st[-1]['t'] if st else None,
                **{'n_'+k:len(data.get(k,[])) for k in STREAMS})

class Recorder:
    def __init__(self,max_points=1500,map_size=None):
        self.max_points=max_points; self.map_size=map_size; self.goal=None
        # Save-only callers retain full history; bounded buffers require a writer.
        for name in STREAMS: setattr(self,name,deque())
        self.events=deque(maxlen=512); self.persistent=[]; self.persistent_unknown=[]; self.persistent_bbox=None
        self._lock=threading.RLock(); self._seq=0; self._queue=queue.Queue(maxsize=2048)
        self._thread=None; self._path=None; self.error=None; self.closed=False; self._manifest=None
    def start(self,path,cfg=None):
        self._check()
        if self.closed: raise RuntimeError('recording closed')
        if self._thread: return
        if self._seq: raise RuntimeError('start must precede recorded events; use save for legacy history')
        self._path=Path(path).parent/'manifest.json'; self._path.parent.mkdir(parents=True,exist_ok=True)
        if self._path.exists(): raise FileExistsError('Do not overwrite an existing recording')
        from .backend import BUILD_HASH,native
        self._manifest={'schema_version':2,'recording_id':self._path.parent.name,'chunks':[],'complete':False,
            'goal':_jsonable(self.goal),'map_size':self.map_size,'config':cfg,'kernel_hash':BUILD_HASH,'native':native is not None,
            'counts':{k:0 for k in STREAMS},'t0':None,'t1':None}
        atomic_json(self._path,self._manifest)
        for name in STREAMS: setattr(self,name,deque(maxlen=512))
        self._thread=threading.Thread(target=self._writer,name='record-writer',daemon=True); self._thread.start()
    def _check(self):
        if self.error: raise RuntimeError('Recording writer failed; previous manifest retained') from self.error
    def _append(self,kind,body):
        body = copy.deepcopy(body)
        with self._lock:
            self._check()
            if self.closed: raise RuntimeError('recording closed')
            self._seq+=1; body=dict(body,seq=self._seq)
            (getattr(self,kind) if kind in STREAMS else self.events).append(body)
            if self._thread:
                try: self._queue.put_nowait({'kind':kind,'data':body})
                except queue.Full as e: self.error=e; raise RuntimeError('recording queue overflow') from e
    def set_goal(self,goal): self.goal=_vec(goal)
    def record_state(self,t,pos,vel,acc,yaw=0.):
        try: t=float(t); yaw=float(yaw)
        except (ValueError,TypeError): return
        if not np.isfinite(t): return
        self._append('states',dict(t=t,p=_vec(pos),v=_vec(vel),a=_vec(acc),yaw=yaw if np.isfinite(yaw) else 0.))
    def record_points(self,t,pts):
        t = _timestamp(t)
        if t is None: return
        a=_array(pts)
        if len(a)>self.max_points: a=a[np.linspace(0,len(a)-1,self.max_points,dtype=int)]
        self._append('points',dict(t=t,pts=a.ravel()))
    def record_map(self,t,occ,unk=None):
        t = _timestamp(t)
        if t is None: return
        self._append('maps',dict(t=t,occ=_array(occ).ravel(),unk=_array(unk).ravel()))
    def record_plan(self,t,goal,path,traj,ctrl):
        t = _timestamp(t)
        if t is None: return
        self._append('plans',dict(t=t,goal=_vec(goal),path=_array(path),traj=_array(traj),ctrl=_array(ctrl)))
    def record_event(self,kind,data): self._append(kind,data)
    def set_persistent(self,points,unknown=None,bbox=None):
        with self._lock:
            self.persistent=_array(points).ravel(); self.persistent_unknown=_array(unknown).ravel(); self.persistent_bbox=bbox
    def latest_state(self):
        with self._lock: return self.states[-1] if self.states else None
    def latest_points(self):
        with self._lock: return self.points[-1] if self.points else None
    def latest_plan(self):
        with self._lock: return self.plans[-1] if self.plans else None
    def to_dict(self):
        with self._lock:
            data={k:list(getattr(self,k)) for k in STREAMS}
            data.update(goal=self.goal,map_size=self.map_size,persistent=self.persistent,
                        persistent_unknown=self.persistent_unknown,persistent_bbox=self.persistent_bbox)
        return _jsonable(data)
    def meta(self):
        if self._manifest:
            m=self._manifest
            return dict(goal=m['goal'],t0=m['t0'],t1=m['t1'],complete=m['complete'],**{'n_'+k:m['counts'][k] for k in STREAMS})
        return _meta(self.to_dict())
    def summary(self): return {k:v for k,v in self.meta().items() if k.startswith('n_')}
    def snapshot_live(self):
        with self._lock:
            latest={k:(getattr(self,k)[-1] if getattr(self,k) else None) for k in STREAMS}
            geometry=next((e for e in reversed(self.events) if 'shape' in e and 'origin' in e),None)
        st=latest['states']; pt=latest['points']
        return _jsonable(dict(empty=st is None,goal=self.goal,map_size=self.map_size,state=st,
            point=pt,pts=pt['pts'] if pt else [],plan=latest['plans'],map=latest['maps'],geometry=geometry))
    def _writer(self):
        batch=[]; last=time.monotonic(); map_previous=None; map_count=0
        try:
            while True:
                try: item=self._queue.get(timeout=.25)
                except queue.Empty: item='timer'
                finish=item is None
                flush=isinstance(item,threading.Event)
                if isinstance(item,dict): batch.append(item)
                if batch and (finish or flush or len(batch)>=256 or time.monotonic()-last>=1.):
                    # Map keyframes every ten snapshots; deltas include removals.
                    serialized=[]
                    for event in batch:
                        event=_jsonable(event)
                        if event['kind']=='maps':
                            body=event['data']; current={k:set(map(tuple,np.asarray(body[k]).reshape(-1,3))) for k in ('occ','unk')}
                            if map_previous is not None and map_count%10:
                                event={'kind':'map_delta','data':dict(t=body['t'],seq=body['seq'],
                                    **{k+'_add':[list(v) for v in sorted(current[k]-map_previous[k])] for k in current},
                                    **{k+'_remove':[list(v) for v in sorted(map_previous[k]-current[k])] for k in current})}
                            map_previous=current; map_count+=1
                        serialized.append(event)
                    raw=json.dumps(serialized,ensure_ascii=False,allow_nan=False,separators=(',',':')).encode('utf-8')
                    name=f'chunk_{len(self._manifest["chunks"]):06d}.json.gz'; target=self._path.parent/name
                    tmp=target.with_suffix('.tmp'); payload=gzip.compress(raw,compresslevel=1)
                    with tmp.open('wb') as f: f.write(payload); f.flush(); os.fsync(f.fileno())
                    os.replace(tmp,target)
                    # Build the next manifest separately; publish only after the atomic commit.
                    m=json.loads(json.dumps(self._manifest)); m['chunks'].append(dict(file=name,sha256=hashlib.sha256(payload).hexdigest(),events=len(batch)))
                    for event in batch:
                        kind=event['kind']; body=event['data']
                        if kind in STREAMS: m['counts'][kind]+=1
                        if kind=='states':
                            m['t0']=body['t'] if m['t0'] is None else m['t0']; m['t1']=body['t']
                    atomic_json(self._path,m); self._manifest=m; batch=[]; last=time.monotonic()
                if flush: item.set()
                if finish:
                    m=dict(self._manifest,complete=not bool(self.error),error=str(self.error) if self.error else None)
                    atomic_json(self._path,m); self._manifest=m; return
        except BaseException as e: self.error=e
    def flush(self):
        with self._lock:
            self._check()
            if not self._thread or self.closed: return
            event=threading.Event(); self._queue.put(event,timeout=2)
        deadline = time.monotonic() + 10
        while not event.wait(.05):
            self._check()
            if not self._thread.is_alive(): raise RuntimeError('recording writer stopped before flush')
            if time.monotonic() >= deadline: raise TimeoutError('recording flush timeout')
        self._check()
    def close(self):
        with self._lock:
            if self.closed:
                self._check()
                return
            self.closed=True
            # Closing and accepting events share a lock: nothing may be queued
            # after the end marker, even when a producer races with shutdown.
            if self._thread and self._thread.is_alive(): self._queue.put(None,timeout=2)
        if self._thread:
            self._thread.join(10); self._check()
            if self._thread.is_alive(): raise TimeoutError('record writer shutdown')
    def save(self,path):
        if self._thread:
            if not self.closed: self.flush()
            self._check(); return str(self._path)
        data=self.to_dict()
        # No fallback is allowed to replace a previously complete recording.
        atomic_json(path,data)
        atomic_json(Path(path).parent/'meta.json',dict(_meta(data),name=Path(path).parent.name,recording=Path(path).name))
        return path

def load_recording_dict(path):
    path=Path(path)
    with path.open(encoding='utf-8') as f: meta=json.load(f)
    if 'schema_version' not in meta: return meta
    if meta['schema_version'] != 2: raise ValueError('unsupported recording schema version')
    data={k:[] for k in STREAMS}; data.update(goal=meta.get('goal'),map_size=meta.get('map_size'),persistent=[],persistent_unknown=[],persistent_bbox=None,complete=meta['complete'],error=meta.get('error'),geometry=[],outcomes=[])
    current=None; last_seq=0; chunk_names=set()
    for chunk in meta['chunks']:
        name=chunk['file']
        if not isinstance(name,str) or not name.startswith('chunk_') or not name.endswith('.json.gz') or '/' in name or '\\' in name or ':' in name or Path(name).name!=name: raise ValueError('invalid chunk path')
        if name in chunk_names: raise ValueError('duplicate chunk')
        chunk_names.add(name)
        payload=(path.parent/name).read_bytes()
        if hashlib.sha256(payload).hexdigest()!=chunk['sha256']: raise ValueError('chunk checksum mismatch')
        events = json.loads(gzip.decompress(payload))
        if not isinstance(events,list) or len(events)!=chunk['events']: raise ValueError('chunk event count mismatch')
        for event in events:
            kind=event['kind']; body=event['data']
            seq=body.get('seq')
            if type(seq) is not int or seq <= last_seq: raise ValueError('event sequence is not increasing')
            last_seq=seq
            if kind=='maps':
                current={k:set(map(tuple,np.asarray(body[k]).reshape(-1,3))) for k in ('occ','unk')}
            elif kind=='map_delta':
                if current is None: raise ValueError('delta without keyframe')
                for k in current:
                    current[k].difference_update(map(tuple,body[k+'_remove'])); current[k].update(map(tuple,body[k+'_add']))
                body=dict(t=body['t'],seq=body['seq'],**{k:np.asarray(sorted(v)).ravel().tolist() for k,v in current.items()}); kind='maps'
            if kind in STREAMS: data[kind].append(body)
            elif kind=='map_geometry': data['geometry'].append(body)
            elif kind=='outcome': data['outcomes'].append(body)
    for k in STREAMS:
        if len(data[k])!=meta['counts'][k]: raise ValueError('manifest count mismatch')
    return data

def scan_recordings(root_dir):
    root=Path(root_dir)
    if not root.exists(): return []
    manifests=list(root.rglob('manifest.json'))
    legacy=[p for p in root.rglob('recording.json') if not (p.parent/'manifest.json').exists()]
    return [str(p) for p in sorted(manifests+legacy,key=lambda p:p.stat().st_mtime_ns,reverse=True)]

def read_meta(path,root_dir=None):
    p=Path(path)
    if p.name=='manifest.json':
        m=json.loads(p.read_text(encoding='utf-8'))
        meta=dict(goal=m['goal'],t0=m['t0'],t1=m['t1'],complete=m['complete'],**{'n_'+k:m['counts'][k] for k in STREAMS})
    else: meta=_meta(load_recording_dict(p))
    meta.update(name=os.path.relpath(p.parent,root_dir) if root_dir else p.parent.name,path=str(p),bytes=p.stat().st_size)
    return meta

class RecordingLibrary:
    def __init__(self,root_dir):
        self.root_dir=root_dir; self._paths=[]; self._cache=OrderedDict(); self._lock=threading.RLock(); self.cache_bytes=128*1024*1024
    def scan(self):
        with self._lock: self._paths=scan_recordings(self.root_dir)
        return self.list()
    def list(self):
        with self._lock:
            self._paths=scan_recordings(self.root_dir)
            entries = []
            for p in self._paths:
                try: entry = read_meta(p,self.root_dir)
                except (OSError, ValueError, KeyError, TypeError) as exc:
                    entry = dict(path=p, name=os.path.relpath(Path(p).parent,self.root_dir), error=str(exc))
                entry['id'] = hashlib.sha256(os.path.relpath(p,self.root_dir).encode()).hexdigest()[:16]
                entries.append(entry)
            return entries
    def count(self): return len(self.list())
    def get(self,index):
        with self._lock:
            if isinstance(index,int):
                if not 0<=index<len(self._paths): return None
                p=self._paths[index]
            else:
                p=next((m['path'] for m in self.list() if m['id']==index),None)
                if p is None: return None
            st=Path(p).stat(); key=(p,st.st_mtime_ns,st.st_size)
            if key not in self._cache:
                value=load_recording_dict(p); size=len(json.dumps(value))
                for old in list(self._cache):
                    if old[0]==p: del self._cache[old]
                while self._cache and sum(v[1] for v in self._cache.values())+size>self.cache_bytes: self._cache.popitem(last=False)
                if size<=self.cache_bytes: self._cache[key]=(value,size)
                return value
            self._cache.move_to_end(key); return self._cache[key][0]
