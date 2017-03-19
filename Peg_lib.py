import numpy as np
#TODO Calculate the launch window and azimuth here, instead in the main script

#For ease of use
#Trigon function in degrees :)
def r2d(x):
    return x*180/np.pi

def d2r(x):
    return x*np.pi/180

def cosd(x):
    return np.cos(d2r(x))

def acosd(x):
    return r2d(np.arccos(x))

def sind(x):
    return np.sin(d2r(x))

def asind(x):
    return r2d(np.arcsin(x))

def tand(x):
    return np.tan(d2r(x))

def atand(x):
    return r2d(np.arctan(x))

def atan2d(x,y):
    return r2d(np.arctan2(x,y))

#Another simplified function, and yeah i am lazy
#This one is for vector operation
def norm(x):
    return np.linalg.norm(x)

def unit(x):
    if norm(x) == 0:
        return x
    else :
        return x/norm(x)

def cross(x,y):
    return np.cross(x,y)

def dot(x,y):
    return np.vdot(x,y)

def vang(x,y):
    x = unit(x)
    y = unit(y)
    return acosd(np.clip(dot(x,y),-1,1))

#This is the black magic
def peg(vessel,tgt,olda,oldb,oldt):
    cycle = 0.1
    mu = vessel.orbit.body.gravitational_parameter
    alt = vessel.orbit.radius
    vt = vessel.flight(vessel.orbit.body.non_rotating_reference_frame).horizontal_speed
    vr = vessel.flight(vessel.orbit.body.non_rotating_reference_frame).vertical_speed
    acc = vessel.thrust/vessel.mass
    ve = vessel.specific_impulse*9.80655

    tau = ve/acc

    if olda == 0 and oldb == 0:
        b0 = -ve*np.log(1 - oldt/tau)
        b1 = b0*tau - ve*oldt
        c0 = b0*oldt - b1
        c1 = c0*tau - 0.5*ve*oldt**2

        z0 = -vr
        z1 = tgt - alt - vr*oldt

        matA = np.array([[b0,b1],[c0,c1]])
        matB = np.array([z0,z1])
        matX = np.linalg.solve(matA,matB)

        olda = matX[0]
        oldb = matX[1]

    angM = norm(cross([alt,0,0],[vr,vt,0]))
    tgtV = np.sqrt(mu/tgt)
    tgtM = norm(cross([tgt,0,0],[0,tgtV,0]))
    dMom = tgtM - angM

    c = (mu/tgt**2 - tgtV**2/tgt) / (acc / (1 - oldt/tau))
    f_r_T = olda + oldb*oldt + c
    c = (mu/alt**2 - vt**2/alt) / acc
    f_r = olda + c
    f_r_dot = (f_r_T - f_r) / oldt

    f_theta = 1 - 0.5*f_r**2
    f_theta_dot = -1*f_r*f_r_dot
    f_theta_2dot = -0.5*f_r_dot**2

    avgR = (alt + tgt)/2
    deltav = dMom/avgR + ve*(oldt - cycle)*(f_theta_dot + f_theta_2dot*tau) + 0.5*f_theta_2dot*ve*(oldt - cycle)**2
    deltav = deltav / (f_theta + f_theta_dot*tau + f_theta_2dot*tau**2)

    t = tau*(1 - np.exp(-deltav/ve))

    if t >= 5:
        b0 = -ve*np.log(1 - t/tau)
        b1 = b0*tau - ve*t
        c0 = b0*t - b1
        c1 = c0*tau - 0.5*ve*t**2

        z0 = -vr
        z1 = tgt - alt - vr*t

        matA = np.array([[b0,b1],[c0,c1]])
        matB = np.array([z0,z1])
        matX = np.linalg.solve(matA,matB)

        a = matX[0]
        b = matX[1]
    else:
        a = olda
        b = oldb

    return np.array([a,b,c,t])
