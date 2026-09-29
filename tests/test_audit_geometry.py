import numpy as np
import pytest
from config import Config
from nav.occupancy import OccupancyMap
from nav.persistent_map import PersistentMap
from nav.geometry import distance_query
from nav.pipeline import NavigationPipeline
from nav import raycast, astar
from nav.backend import native

def test_occlusion_and_new_evidence():
    m=OccupancyMap(15,15,15,1); m.data[7,3:12,3:12]=2
    m.mark_visible_free([3.5,7.5,7.5],20)
    assert m.data[9,7,7]==0
    pm=PersistentMap(1,4)
    for state in (2,1): pm.merge_from_local(np.full((1,1,1),state,dtype='int8'),np.zeros(3),1)
    assert pm.query_voxel(0,0,0)==1

@pytest.mark.parametrize('fast',[False,True])
def test_ray_order(fast,monkeypatch):
    if fast and native is None: pytest.skip('native not built')
    monkeypatch.setattr(raycast,'USING_FAST',fast)
    cfg=Config(res=1,max_range=20); hits=np.array([[3.2,.5,.5],[6.2,.5,.5]])
    maps=[]
    for pts in (hits,hits[::-1]):
        m=OccupancyMap(10,3,3,1); raycast.integrate(m,[.5,.5,.5],pts,cfg); maps.append(m.data.copy())
    np.testing.assert_array_equal(*maps)
    assert m.data[3,0,0]==2
    raycast.integrate(m,[.5,.5,.5],hits[1:],cfg)
    assert m.data[3,0,0]==1

def test_distance_axes_centres_gradient_and_bounds():
    i,j,k=np.indices((5,5,5)); d=(100*i+10*j+k).astype('float32')
    v,g,valid=distance_query([[1.5,1.5,1.5],[1.7,2.3,2.4],[-.1,2,2]],d,np.zeros(3),1)
    np.testing.assert_allclose(v[:2],[111,139.9],atol=1e-5)
    np.testing.assert_allclose(g[:2],[[100,10,1]]*2)
    assert not valid[-1] and v[-1]==0
    assert distance_query([[.1,.1,.1]],np.zeros((3,3,3)),np.zeros(3),1)[0][0]==0
    if native:
        for p in ([1.5,1.5,1.5],[1.7,2.3,2.4],[.1,.1,.1],[-.1,0,0]):
            assert native.distance_at(np.array(p),d,np.zeros(3),1)==pytest.approx(distance_query(p,d,np.zeros(3),1)[0][0])

def test_coarse_unknown_and_resolution():
    a=np.zeros((2,2,2),dtype='int8'); a[0,0,0]=1
    assert NavigationPipeline._coarsen(a,np.zeros(3),.2,.4).data.item()==0
    with pytest.raises(ValueError): NavigationPipeline._coarsen(a,np.zeros(3),.2,.5)

def test_projection_label_zero():
    m=OccupancyMap(5,5,5,1); m.data.fill(1); m.data[3,3,3]=2
    d=np.full(m.data.shape,5.); d[1,1,1]=0; start=np.array([1.5]*3)
    np.testing.assert_array_equal(NavigationPipeline._project_goal_free(m,d,start,[3.5]*3,d_min=.8),start)

@pytest.mark.parametrize('fast',[False,True])
def test_astar_no_corner_cut_or_start_relocation(fast,monkeypatch):
    if fast and native is None: pytest.skip('native not built')
    monkeypatch.setattr(astar,'USING_FAST',fast)
    m=OccupancyMap(2,2,1,1); m.data.fill(2); m.data[0,0,0]=m.data[1,1,0]=1
    d=np.full(m.data.shape,5.,dtype='float32'); p=astar.AStarPlanner(Config(res=1))
    assert p.plan(m,d,[.5,.5,.5],[1.5,1.5,.5]) is None
    assert p.plan(m,d,[.5,1.5,.5],[1.5,1.5,.5]) is None
