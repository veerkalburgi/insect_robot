import numpy as np
import math
from scipy import interpolate

step_num = 40
step_trip = 0.15
offset = 0.5
step_angle = math.pi / 6
turn_para = 2
l1 = 0.148
l2 = 0.219
#l3 = 0.35
radio = 40


def rpy_gait(RPY_angle_old, RPY_angle_new_traj):
    gait_data = np.zeros((radio, 8))
    rate = 3
    RPY_angle_new_traj = np.array(RPY_angle_new_traj)
    traj = np.vstack((RPY_angle_old, RPY_angle_new_traj))
    RPY_B_traj = np.zeros((step_num,3))

    for i in range (3):
        RPY_B_traj[:, i] = interpolate_method(traj[:, i].tolist())

    for t in range(RPY_B_traj.shape[0]):
        R = RPY_B_traj[t, 0]
        P = RPY_B_traj[t, 1]
        Y = RPY_B_traj[t, 2]

        pos_data = rpy_control(R, P, Y)

        lf_theta1, lf_theta2 = leg_ikine(pos_data[0,0], pos_data[0,1])
        rf_theta1, rf_theta2 = leg_ikine(pos_data[1,0], pos_data[1,1])
        lb_theta1, lb_theta2 = leg_ikine(pos_data[2,0], pos_data[2,1])
        rb_theta1, rb_theta2 = leg_ikine(pos_data[3,0], pos_data[3,1])

        gait_data[t, 0] = lf_theta1
        gait_data[t, 1] = lf_theta2
        #gait_data[t, 2] = lf_theta3

        gait_data[t, 2] = rf_theta1
        gait_data[t, 3] = rf_theta2
        #gait_data[t, 5] = rf_theta3

        gait_data[t, 4] = lb_theta1
        gait_data[t, 5] = lb_theta2
        #gait_data[t, 8] = lb_theta3

        gait_data[t, 6] = rb_theta1
        gait_data[t, 7] = rb_theta2
        #gait_data[t, 11] = rb_theta3
    return rate, gait_data, RPY_B_traj

def rpy_adj(RPY_angle_new_traj):
    gait_data = np.zeros((1, 12))
    rate = 3
    RPY_angle_new_traj = np.array(RPY_angle_new_traj)

    R = RPY_angle_new_traj[0]
    P = RPY_angle_new_traj[1]
    Y = RPY_angle_new_traj[2]

    pos_data = rpy_control(R, P, Y)

    lf_theta1, lf_theta2 = leg_ikine(pos_data[0,0], pos_data[0,1])
    rf_theta1, rf_theta2 = leg_ikine(pos_data[1,0], pos_data[1,1])
    lb_theta1, lb_theta2 = leg_ikine(pos_data[2,0], pos_data[2,1])
    rb_theta1, rb_theta2 = leg_ikine(pos_data[3,0], pos_data[3,1])

    gait_data[0, 0] = lf_theta1
    gait_data[0, 1] = lf_theta2
    #gait_data[0, 2] = lf_theta3

    gait_data[0, 2] = rf_theta1
    gait_data[0, 3] = rf_theta2
    #gait_data[0, 5] = rf_theta3

    gait_data[0, 4] = lb_theta1
    gait_data[0, 5] = lb_theta2
    #gait_data[0, 8] = lb_theta3

    gait_data[0, 6] = rb_theta1
    gait_data[0, 7] = rb_theta2
    #gait_data[0, 11] = rb_theta3
    return rate, gait_data

def leg_ikine(x, y, z):
    #theta1 = math.atan2(y, x) + math.atan2(l1, -(x**2 + y**2 - l1**2)**0.5)
    #c1 = math.cos(theta1)
    #s1 = math.sin(theta1)
    #c3 = (x**2 + y**2 + z**2 - l1**2 - l2**2 - l3**2) / (2 * l2 * l3)
    #s3 = (1 - c3**2)**0.5
    #theta3 = math.atan2(s3, c3)
    #s2p = (l3 * s3) / ((y * s1 + x * c1)**2 + z**2)**0.5
    #c2p = (1 - s2p**2)**0.5
    #theta2p = -math.atan2(s2p, c2p)
    #thetap = -math.atan2(z, -(y * s1 + x * c1))
    #theta2 = theta2p - thetap
    #return theta1 - math.pi, theta2, theta3
    c2 = (x**2 +y**2 -l1**2 -l2**2)/(2*l1*l2)
    s2 = (1-c2**2)**0.5
    theta1 = math.atan2(s2,c2)
    K1 = l1+(c2*l2)
    K2 = s2*l2
    theta0 = math.atan2(y,x)-math.atan2(K2,K1)
     return theta0,theta1


def rpy_control(R, P, Y):
    # Define the structure paramesters of robot
    l = 0.8 # The length of platform.
    w = 0.8 # The width of platform.

    # Rebuild the Euler-angle.
    R = R
    P = P
    Y = Y

    # Foot position vector
    det = np.array([[ l/2, -0.55, -0.55],
                    [ l/2, -0.55,  0.55],
                    [-l/2, -0.55, -0.55],
                    [-l/2, -0.55,  0.55],])

    pos_data = np.zeros((4, 3))
    for i in range(4):
        detx = det[i, 0]
        dety = det[i, 1]
        detz = det[i, 2]
        rotx = np.mat([[ 1,       0,       0     ],
                       [ 0,       math.cos(R), -math.sin(R)],
                       [ 0,       math.sin(R),  math.cos(R)]])

        rotz = np.mat([[ math.cos(P), -math.sin(P),  0     ],
                       [ math.sin(P),  math.cos(P),  0     ],
                       [ 0,       0,       1     ]])

        roty = np.mat([[ math.cos(Y),  0,      -math.sin(Y)],
                       [ 0,       1,       0     ],
                       [ math.sin(Y),  0,       math.cos(Y)]])
        det_vec = np.mat([[detx], [dety], [detz]])
        off_vec = np.mat([[ l/2, l/2, -l/2, -l/2],
                          [ 0,   0,    0,    0  ],
                          [-w/2, w/2, -w/2, w/2 ]])
        pos_vec = rotx * roty * rotz * det_vec - off_vec[:, i]
        # Converte the global coordinate into the kinematics coordinates of each leg.
        x = -pos_vec[1]
        z = pos_vec[0]
        if (i%2)==0 :
            y = -pos_vec[2]
        else:
            y = pos_vec[2]
        pos_data[i, :] = np.array([x, y, z]).T
    return pos_data

def interpolate_method(y):
    t = len(y)
    x = np.linspace(0, t, t)
    xnew=np.linspace(0, t, step_num)
    #pl.plot(x,y,"ro")
    for kind in ["slinear"]:# choose interpolate method:
        #"nearest","zero"
        # slinear
        #"quadratic","cubic"
        f=interpolate.interp1d(x,y,kind=kind)
        # 'slinear', 'quadratic' and 'cubic' refer to a spline interpolation of first, second or third order)
        ynew=f(xnew)
        #pl.plot(xnew,ynew,label=str(kind))
    #pl.legend(loc="lower right")
    #pl.show()
    return ynew
