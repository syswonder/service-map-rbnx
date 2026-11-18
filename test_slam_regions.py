import json
from pathlib import Path
import pytest
import yaml
from mapping_rbnx import slam_regions as regions

@pytest.mark.parametrize('bounds', [None, [], [1,2,3], [1,2,1,4], [3,2,1,4],
    [0,0,float('nan'),1], [0,0,float('inf'),1], [False,0,1,1], ['0',0,1,1]])
def test_invalid_regions_rejected(bounds):
    with pytest.raises(ValueError):
        regions.bounds_of(bounds)

def test_valid_bounds():
    assert regions.bounds_of([-2,0,3,4]) == [-2.,0.,3.,4.]

def base(tmp_path, engine='slam_toolbox'):
    source=tmp_path/'base';source.mkdir()
    (source/'meta.yaml').write_text(yaml.safe_dump({'engine':engine}))
    (source/'occupancy.yaml').write_text('resolution: 0.05\n')
    for name in regions.GRAPH_FILES:(source/name).write_bytes(b'original')
    return source

def test_rtabmap_rejected_without_running_native(tmp_path,monkeypatch):
    source=base(tmp_path,'rtabmap')
    monkeypatch.setattr(regions.subprocess,'run',lambda *a,**k:pytest.fail('native tool called for RTAB-Map'))
    with pytest.raises(ValueError,match='only supported'):
        regions.prepare(tmp_path,'base','derived',[0,0,1,1])
    assert not (tmp_path/'derived').exists()
    assert (source/regions.GRAPH_FILES[0]).read_bytes()==b'original'

def test_failure_keeps_source_and_removes_stage(tmp_path,monkeypatch):
    source=base(tmp_path)
    def fail(*a,**kw):raise RuntimeError('unreadable native graph')
    monkeypatch.setattr(regions,'inspect_graph',fail)
    with pytest.raises(RuntimeError,match='unreadable'):
        regions.prepare(tmp_path,'base','derived',[0,0,1,1])
    assert not (tmp_path/'derived').exists()
    assert not [p for p in tmp_path.glob('.region-*') if p.is_dir()]
    assert all((source/n).read_bytes()==b'original' for n in regions.GRAPH_FILES)

def test_existing_map_not_overwritten(tmp_path):
    base(tmp_path);destination=tmp_path/'derived';destination.mkdir();(destination/'user-data').write_text('keep')
    with pytest.raises(ValueError,match='immutable'):
        regions.prepare(tmp_path,'base','derived',[0,0,1,1])
    assert (destination/'user-data').read_text()=='keep'

def test_path_escape_rejected(tmp_path):
    with pytest.raises(ValueError):regions.prepare(tmp_path,'../other','derived',[0,0,1,1])

def test_local_start_must_use_retained_observations(tmp_path):
    from mapping_rbnx.localizers import _Grid
    grid=_Grid(4,4,bytearray(16),bytearray([1]*16),1.,[0,0,0])
    (tmp_path/'local-update.json').write_text(json.dumps({'bounds':[1,1,2,2],'margin_m':.05}))
    assert 'outside' in regions.retained_start(tmp_path,[1.5,1.5,0],grid)
    assert regions.retained_start(tmp_path,[.5,.5,0],grid) is None
    grid.known[0]=0
    assert 'known free' in regions.retained_start(tmp_path,[.5,.5,0],grid)

def test_local_rebuild_page_assets_are_served():
    from mapping_rbnx.webui import _static
    code,kind,body=_static('local-update.html')
    assert code==200 and 'text/html' in kind and b'local-update.js' in body
    code,kind,body=_static('local-update.js')
    assert code==200 and 'javascript' in kind and body
