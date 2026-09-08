"""Analytical sanity checks for the independent VTU force/work reconstruction."""
import base64
from pathlib import Path
import tempfile
import unittest
import xml.etree.ElementTree as ET
import numpy as np
from measure_fem import measure


def fixture(path, F):
    X=np.array([[0,0,0],[1,0,0],[0,1,0],[1,1,0],
                [0,0,1],[1,0,1],[0,1,1],[1,1,1]],dtype=float)
    tets=np.array([[0,1,3,7],[0,3,2,7],[0,2,6,7],
                   [0,6,4,7],[0,4,5,7],[0,5,1,7]],dtype=np.int64)
    # Deliberately duplicate all output vertices by element, as PolyFEM does.
    X=X[tets.flatten()]
    root=ET.Element('VTKFile',header_type='UInt64')
    def array(name, values, kind='Float64'):
        data=np.asarray(values,dtype={'Float64':'<f8','Int64':'<i8','UInt8':'u1'}[kind])
        attrs=dict(type=kind,format='binary',NumberOfComponents=str(data.shape[1] if data.ndim==2 else 1))
        if name:attrs['Name']=name
        node=ET.SubElement(root,'DataArray',attrs)
        raw=data.tobytes();node.text=base64.b64encode(len(raw).to_bytes(8,'little')+raw).decode()
    array(None,X);array('displacement',X@F.T-X)
    for i in range(3):array(f'F_{i+1}',np.tile(F[:,i],(len(X),1)))
    array('connectivity',np.arange(24),'Int64')
    array('offsets',np.arange(4,25,4),'Int64')
    array('types',np.full(6,10),'UInt8')
    ET.ElementTree(root).write(path)


class ReconstructionTests(unittest.TestCase):
    def test_rigid_rotation_has_zero_internal_energy_and_reaction(self):
        angle=.4
        F=np.array([[np.cos(angle),-np.sin(angle),0],
                    [np.sin(angle),np.cos(angle),0],[0,0,1]])
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'step_0.vtu';fixture(path,F)
            r=measure(path,1e7,.45,0)
        self.assertLess(abs(r['elastic_energy']),1e-8)
        self.assertLess(np.linalg.norm(r['top_reaction']),1e-8)
        self.assertEqual(r['node_count'],8)
        self.assertAlmostEqual(r['volume'],1.)

    def test_reaction_is_work_conjugate_to_prescribed_stretch(self):
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'step_0.vtu';rows=[]
            eps=1e-6
            for stretch in (.75-eps,.75,.75+eps):
                fixture(path,np.diag([1.,1.,stretch]))
                rows.append(measure(path,1e7,.45,0))
            derivative=(rows[2]['elastic_energy']-rows[0]['elastic_energy'])/(2*eps)
        self.assertLess(abs(derivative-rows[1]['top_reaction'][2])/1e7,1e-8)
        self.assertLess(rows[1]['elastic_net_force_norm']/1e7,1e-12)
        self.assertAlmostEqual(rows[1]['min_det_F'],.75)


if __name__=='__main__':unittest.main()
