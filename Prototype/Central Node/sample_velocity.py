import numpy as np
def sampling(xt_1,jacobian=False):
    xt = xt_1[0]
    xt_= xt_1[1]
    x = xt[0]
    y = xt[1]
    theta = xt[2]
    x_ = xt_[0]
    y_ = xt_[1]
    theta_ = xt_[2]
    a1 = 0.25 #0.25 rot_rot degrees/degrees
    a2 = 0.009 # 5 rot_trans degrees/meters
    a3 = 0.001 # 0.01 trans_trans meters/metres
    a4 = 0.0001 # 0.0001 trans_rot metres/deg

    rot1 = np.math.atan2(y_-y,x_-x)-theta
    trans = np.math.sqrt((x-x_)**2+(y-y_)**2)
    rot2 = theta_ - theta -  rot1

    rot1_std = a1*abs(rot1)+a2*abs(trans)
    trans_std = a3*abs(trans)+a4*abs(rot1+rot2)
    rot2_std = a1*abs(rot2)+a2*abs(trans)

    rot1_ = rot1 + np.random.normal(0,rot1_std) #sample(a1*rot1+a2*trans)
    trans_ = trans + np.random.normal(0,trans_std) #sample(a3*trans+a4*(rot1+rot2))
    rot2_ = rot2 + np.random.normal(0,rot2_std) #sample(a1*rot2+a2*trans)

    x_t = x + trans_*np.math.cos(theta+rot1_)
    y_t = y + trans_*np.math.sin(theta+rot1_)
    theta_t = theta + rot1_ + rot2_
    if jacobian:
        Pt = np.array([[rot1_std,0,0],[0,trans_std,0],[0,0,rot2_std]])
        Gs = np.array([[1,0,-trans_*np.math.sin(theta+rot1_)],[0,1,trans_*np.math.cos(theta+rot1_)],[0,0,1]])
        xt = np.array([x_t,y_t,theta_t])
        return xt,Pt,Gs
    else:
        return np.array([x_t,y_t,theta_t])
def sample(b):
    e = sum([np.random.uniform(-1,1) for x in range(12)])
    return (b/6)*e
def velocity():
    pass
