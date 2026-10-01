import numpy as np
import pytest
from wam_kinesthetic_dmps import replay


def test_export_slows_time_without_changing_path(tmp_path, monkeypatch):
    t=np.array([0.0,0.01,0.02])
    q=np.arange(21,dtype=float).reshape(3,7)/100
    monkeypatch.setattr(replay,"rollout",lambda model:(t,q))
    path=tmp_path/"trajectory.csv"
    duration,start=replay.export_trajectory({},path,3)
    saved=np.loadtxt(path,delimiter=",",skiprows=1)
    assert path.read_text().splitlines()[0]==replay.HEADER
    np.testing.assert_allclose(saved[:,0],t*3)
    np.testing.assert_allclose(saved[:,1:],q)
    np.testing.assert_allclose(start,q[0])
    assert duration==pytest.approx(0.06)
    with pytest.raises(FileExistsError):
        replay.export_trajectory({},path)


@pytest.mark.parametrize("slowdown",[0,-1,0.5,float("nan"),float("inf")])
def test_invalid_slowdown(tmp_path,slowdown):
    with pytest.raises(ValueError):
        replay.export_trajectory({},tmp_path/"bad.csv",slowdown)
