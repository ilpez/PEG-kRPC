import Global
import numpy as np

conn = Global.conn
space_center = Global.space_center
vessel = Global.vessel
mu = Global.mu
g0 = Global.g0
# For ease of use


# Trigon function in degrees :)


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


def atan2d(x, y):
    return r2d(np.arctan2(x, y))


# Another simplified function, and yeah i am that lazy
# This one is for vector operation
def norm(x):
    return np.linalg.norm(x)


def unit(x):
    if norm(x) == 0:
        return x
    else:
        return x/norm(x)


def cross(x, y):
    return np.cross(x, y)


def dot(x, y):
    return np.vdot(x, y)


def vang(x, y):
    x = unit(x)
    y = unit(y)
    return acosd(np.clip(dot(x, y), -1, 1))

# Calculate target_parameter


def target_parameter(apoapsis,
                     periapsis,
                     inclination,
                     lan,
                     slip):
    orbref = vessel.orbit.body.non_rotating_reference_frame
    # surfref = vessel.orbit.body.reference_frame
    local_x = [1, 0, 0]
    local_y = [0, 1, 0]
    local_z = [0, 0, 1]
    apoapsis *= 1000
    periapsis *= 1000
    apoapsis += vessel.orbit.body.equatorial_radius
    periapsis += vessel.orbit.body.equatorial_radius

    semimajor_axis = (apoapsis + periapsis)/2
    # ecc = (apoapsis - periapsis)/(apoapsis + periapsis)
    velocity = np.sqrt((mu*apoapsis)/(periapsis*semimajor_axis))
    radius = periapsis

    if np.absolute(inclination) < vessel.flight().latitude:
        azimuth = 90
    else:
        beta_inertial = asind(cosd(inclination)/cosd(vessel.flight().latitude))
        if inclination < 0:
            if beta_inertial <= 90:
                beta_inertial = 180 - beta_inertial
            elif beta_inertial >= 270:
                beta_inertial = 540 - beta_inertial
        earth_velocity = (vessel.orbit.body.rotational_speed *
                          vessel.orbit.body.equatorial_radius)
        velocity_x = (velocity*sind(beta_inertial) - earth_velocity *
                      cosd(vessel.flight().latitude))
        velocity_y = (velocity *
                      cosd(beta_inertial))
        azimuth = atan2d(velocity_x, velocity_y)

    relative_longitude = asind(tand(vessel.flight().latitude) /
                               tand(inclination))
    if inclination < 0:
        relative_longitude = 180 + relative_longitude
    # earth_meridian = vessel.orbit.body.msl_position(0, 0, surfref)
    prime_meridian = vessel.orbit.body.msl_position(0, 0, orbref)
    rotational_angle = atan2d(dot(local_z, prime_meridian),
                              dot(local_x, prime_meridian))
    if rotational_angle < 0:
        rotational_angle += 360
    print(rotational_angle)
    geo_longitude = lan + relative_longitude - rotational_angle
    geo_longitude = np.mod(geo_longitude + 360, 360)
    node_angle = geo_longitude - vessel.flight().longitude
    node_angle = np.mod(node_angle + 360 + slip, 360)
    launch_time = (node_angle/360) * vessel.orbit.body.rotational_period

    return np.array([velocity, radius, azimuth, launch_time])

# This is the black magic


def peg(cycle,
        tgtV,
        tgt,
        olda,
        oldb,
        oldt):
    alt = vessel.orbit.radius
    vt = vessel.flight(
        vessel.orbit.body.non_rotating_reference_frame).horizontal_speed
    vr = vessel.flight(
        vessel.orbit.body.non_rotating_reference_frame).vertical_speed
    acc = vessel.thrust/vessel.mass
    ve = vessel.specific_impulse*g0

    tau = ve/acc

    if olda == 0 and oldb == 0:
        b0 = -ve*np.log(1 - oldt/tau)
        b1 = b0*tau - ve*oldt
        c0 = b0*oldt - b1
        c1 = c0*tau - 0.5*ve*oldt**2

        z0 = -vr
        z1 = tgt - alt - vr*oldt

        matA = np.array([[b0, b1], [c0, c1]])
        matB = np.array([z0, z1])
        matX = np.linalg.solve(matA, matB)

        olda = matX[0]
        oldb = matX[1]

    angM = norm(cross([alt, 0, 0], [vr, vt, 0]))
    tgtM = norm(cross([tgt, 0, 0], [0, tgtV, 0]))
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
    deltav = dMom/avgR + ve*(oldt - cycle)*(f_theta_dot + f_theta_2dot*tau)
    deltav = deltav + 0.5*f_theta_2dot*ve*(oldt - cycle)**2
    deltav = deltav / (f_theta + f_theta_dot*tau + f_theta_2dot*tau**2)

    t = tau*(1 - np.exp(-deltav/ve))

    if t >= 5:
        b0 = -ve*np.log(1 - t/tau)
        b1 = b0*tau - ve*t
        c0 = b0*t - b1
        c1 = c0*tau - 0.5*ve*t**2

        z0 = -vr
        z1 = tgt - alt - vr*t

        matA = np.array([[b0, b1], [c0, c1]])
        matB = np.array([z0, z1])
        matX = np.linalg.solve(matA, matB)

        a = matX[0]
        b = matX[1]
    else:
        a = olda
        b = oldb

    return np.array([a, b, c, t])


def angle_from_vec(x, ref, angle):
    east = [0, 0, 1]
    north = [0, 1, 0]
    up = [1, 0, 0]
    surface_frame = vessel.surface_reference_frame
    vector = space_center.transform_direction(x, ref, surface_frame)
    if angle == 'pitch':
        out = 90 - vang(up, vector)
    elif angle == 'yaw':
        out = atan2d(dot(east, vector), dot(north, vector))
        if out < 0:
            out += 360
    return out
