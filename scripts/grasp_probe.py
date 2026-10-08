import numpy as np,pinocchio as pin
from rebot_core import Robot,HOME
from reBotArm_control_py.kinematics.inverse_kinematics import solve_ik,IKParams
r=Robot();rng=np.random.default_rng(42)
for theta in [0,np.pi/2,np.pi,-np.pi/2]:
 R=np.array([[0,-np.sin(theta),np.cos(theta)],[0,np.cos(theta),np.sin(theta)],[-1,0,0]],dtype=float)
 xyz=np.array([.35,0,.23]);best=9
 for n in range(30):
  seed=HOME if n==0 else rng.uniform(r.lower,r.upper)
  res=solve_ik(r.pm,r.pm.createData(),r.fid,pin.SE3(R,xyz),seed,IKParams(tolerance=1e-6,max_iter=300),controlled_joints=6)
  best=min(best,res.error)
  if res.success and not r.contacts(res.q,.03):
   print('SUCCESS',theta,res.q.tolist(),flush=True);break
 else: print('FAIL',theta,best,flush=True)
