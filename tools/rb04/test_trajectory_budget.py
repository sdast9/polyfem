"""Independent analytical and negative controls for the saved-trajectory work audit."""
import unittest
import numpy as np
from trajectory_budget import elastic_segment


class ElasticWork(unittest.TestCase):
    def setUp(self):
        self.X=np.array([[0.,0.,0.],[1.,0.,0.],[0.,1.,0.],[0.,0.,1.]])
        self.data=dict(points=self.X,connectivity=np.arange(4),offsets=np.array([4]),types=np.array([10]))

    def test_uniaxial_stretch_closed_form(self):
        # Unit-reference tetrahedron under F=diag(lambda,1,1).
        # P11=mu*(lambda-1/lambda)+Lame*log(lambda)/lambda.
        E,nu,stretch=100.,.3,1.2
        mu=E/(2*(1+nu));lam=E*nu/((1+nu)*(1-2*nu))
        u=np.zeros_like(self.X);u[:,0]=(stretch-1)*self.X[:,0]
        r=elastic_segment(self.data,np.zeros_like(u),u,E,nu)
        energy=(mu/2*(stretch**2-1-2*np.log(stretch))+lam/2*np.log(stretch)**2)/6
        right=(mu*(stretch-1/stretch)+lam*np.log(stretch)/stretch)*(stretch-1)/6
        self.assertAlmostEqual(r['energy_end'],energy,places=12)
        self.assertAlmostEqual(r['right_work'],right,places=12)
        self.assertAlmostEqual(r['path_work']['8'],energy,places=12)
        self.assertLess(abs(r['path_error']['8']),abs(r['path_error']['2']))

    def test_rigid_translation_has_zero_elastic_work(self):
        u=np.tile([.3,-.4,.5],(4,1))
        r=elastic_segment(self.data,np.zeros_like(u),u,100.,.3)
        self.assertLess(abs(r['path_work']['8']),1e-12)
        self.assertLess(abs(r['energy_end']),1e-12)

    def test_inverted_endpoint_is_rejected(self):
        u=np.zeros_like(self.X);u[1,0]=-2
        with self.assertRaisesRegex(ValueError,'Nonpositive'):
            elastic_segment(self.data,np.zeros_like(u),u,100.,.3)

if __name__=='__main__': unittest.main()
