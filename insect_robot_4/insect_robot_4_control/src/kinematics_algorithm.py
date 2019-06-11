import numpy as np
import math
import cmath
import tarj_data as td

l1 = 0.148
l2 = 0.219
#l3 = 0.35
radio = 20   #40
rate = 2


def forward_gait():
    gait_data = np.zeros((radio, 8))
    x_line, y_line, z_line = gait_line()
    rate = 2
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = x_line[t]
            xb = x_line[t + 20]
            yf = y_line[t]
            yb = y_line[t + 20]
            zf = z_line[t]
            zb = z_line[t + 20]
        else:
            xf = x_line[t]
            xb = x_line[t - 20]
            yf = y_line[t]
            yb = y_line[t - 20]
            zf = z_line[t]
            zb = z_line[t - 20]

        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(xf, yf + 0.15, zf)
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(xb, -yb + 0.15, zb)
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(xb, -yb + 0.15, zb)
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(xf, yf + 0.15, zf)

    return rate, gait_data


def backward_gait():
    gait_data = np.zeros((radio, 8))
    x_line, y_line, z_line = gait_line()
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = x_line[t]
            xb = x_line[t + 20]
            yf = y_line[t]
            yb = y_line[t + 20]
            zf = -z_line[t]
            zb = -z_line[t + 20]
        else:
            xf = x_line[t]
            xb = x_line[t - 20]
            yf = y_line[t]
            yb = y_line[t - 20]
            zf = -z_line[t]
            zb = -z_line[t - 20]

        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(xf, yf, zf)
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(xf, yf, zf)
    return rate, gait_data

def turnleft_gait():
    gait_data = np.zeros((radio, 8))
    data = turn_line(1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(data[0, t], data[4, t])
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(data[1, t], data[5, t])
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(data[2, t], data[6, t])
        gait_data[t, 6], gait_data[t, 7] =  leg_ikine(data[3, t], data[7, t])
    return rate, gait_data

def turnright_gait():
    gait_data = np.zeros((radio, 8))
    data = turn_line(-1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(data[0, t], data[4, t])
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(data[1, t], data[5, t])
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(data[2, t], data[6, t])
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(data[3, t], data[7, t])
    return rate, gait_data

def slantleft_gait():
    gait_data = np.zeros((radio, 8))
    data = td.slantleft_gait(1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(data[0, t], data[2, t])
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(data[1, t], data[3, t])
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(data[1, t], data[3, t])
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(data[0, t], data[2, t])
    return rate, gait_data

def slantright_gait():
    gait_data = np.zeros((radio, 8))
    data = td.slantleft_gait(-1)
    for t in range(gait_data.shape[0]):
        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(data[0, t], data[2, t])
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(data[1, t], data[3, t])
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(data[1, t], data[3, t])
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(data[0, t], data[2, t])
    return rate, gait_data


def jump_gait():

    return

def keep_gait():
    gait_data = np.zeros((radio, 8))
    rate = 2
    x_line, y_line, z_line = keep_line()
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = x_line[t]
            xb = x_line[t + 20]
            yf = y_line[t]
            yb = y_line[t + 20]
            zf = z_line[t]
            zb = z_line[t + 20]
        else:
            xf = x_line[t]
            xb = x_line[t - 20]
            yf = y_line[t]
            yb = y_line[t - 20]
            zf = z_line[t]
            zb = z_line[t - 20]

        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(xf, yf, zf)
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(xf, yf, zf)
    return rate, gait_data

def clam_gait():
    gait_data = np.zeros((radio, 8))
    rate = 2
    x_line, y_line, z_line = keep_line()
    for t in range(gait_data.shape[0]):
        if (t < 20):
            xf = 0.4
            xb = 0.4
            yf = 0.15
            yb = 0.15
            zf = 0.1
            zb = 0.1
        else:
            xf = 0.4
            xb = 0.4
            yf = 0.15
            yb = 0.15
            zf = 0.1
            zb = 0.1

        gait_data[t, 0], gait_data[t, 1 ] = leg_ikine(xf, yf, zf)
        gait_data[t, 2], gait_data[t, 3 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 4], gait_data[t, 5 ] = leg_ikine(xb, yb, zb)
        gait_data[t, 6], gait_data[t, 7] = leg_ikine(xf, yf, zf)
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
    c2 = (x**2 + y**2  - l1**2 - l2**2)/(2*l1*l2)
    c2p= abs(c2)
    s2p = (1 - c2p**2)**0.5

    theta1 =math.atan2(s2p,c2p)
    K1 = l1+(c2p*l2)
    K2 = s2p*l2
    theta0 =math.atan2(y,x)-math.atan2(K2,K1)
    return theta0-math.pi,theta1

def gait_line():
    data = td.forward_gait()
    x_line = data[0, :]
    y_line = data[1, :]
    z_line = data[2, :]
    return x_line, y_line, z_line

def keep_line():
    data = td.keep_gait()
    x_line = data[0, :]
    y_line = data[1, :]
    z_line = data[2, :]
    return x_line, y_line, z_line

def turn_line(direction):
    data = td.turn_gait(direction)
    return data

def jump_line():
    xf_line = np.zeros((20))

    return
