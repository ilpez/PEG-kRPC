import krpc
import mkl
import numpy as np
import time
from Peg_lib import acosd,asind,atand,atan2d,cosd,sind,tand,norm,cross,unit,dot,vang,peg

conn = krpc.connect(name = 'Powered Explicit Guidance')
vessel = conn.space_center.active_vessel

orbref = vessel.orbit.body.non_rotating_reference_frame
surfref = vessel.orbit.body.reference_frame

g0 = 9.80655
mu = vessel.orbit.body.gravitational_parameter

target_altitude = 200000 + vessel.orbit.body.equatorial_radius
target_inclination = 51.65
target_lan = 240

if np.absolute(target_inclination) < vessel.flight().latitude:
    azimuth = 90
else :
    beta_inertial = asind(cosd(target_inclination)/cosd(vessel.flight().latitude))
    target_velocity = np.sqrt(mu/target_altitude)
    velocity_x = target_velocity*sind(beta_inertial) - 465*cosd(vessel.flight().latitude)
    velocity_y = target_velocity*cosd(beta_inertial)
    azimuth = atan2d(velocity_x,velocity_y)

relative_longitude = asind(tand(vessel.flight().latitude)/tand(target_inclination))
rotation_angle = vang(vessel.orbit.body.msl_position(0,0,surfref),vessel.orbit.body.msl_position(0,0,orbref))
geo_longitude = target_lan + relative_longitude - rotation_angle
geo_longitude = np.mod(geo_longitude + 360,360)
node_angle = geo_longitude - vessel.flight().longitude
node_angle = np.mod(node_angle - 1 + 360,360)
launch_time = (node_angle/360) * vessel.orbit.body.rotational_period
game_launch_time = conn.space_center.ut + launch_time
conn.space_center.warp_to(game_launch_time - 10)

while (conn.space_center.ut - game_launch_time) < 0 :
    print ('Time to launch %f' %(conn.space_center.ut - game_launch_time))
    time.sleep(1)

vessel.control.throttle = 1
for engine in vessel.parts.engines:
    if not engine.active:
        print ('There is no active engine, checking Propellant condition')
        for engine in engine.part.modules:
            if engine.has_field('Propellant'):
                if engine.get_field('Propellant') == 'Very Stable':
                    print ('Engine is ready')
                    vessel.control.activate_next_stage()
while (vessel.thrust/vessel.mass) < (g0 + 1):
    time.sleep(0.01)

vessel.auto_pilot.engage()
vessel.auto_pilot.target_heading = 0
vessel.auto_pilot.target_pitch = 90
vessel.control.activate_next_stage()
print ('Proceeding Launch')

while True:
    if vessel.flight(surfref).speed > 20:
        vessel.auto_pilot.target_heading = azimuth
    pitch = atand(1000/vessel.flight(surfref).speed)
    vessel.auto_pilot.target_pitch = pitch
    if vessel.flight(surfref).speed > 2700 or vessel.available_thrust == 0:
        vessel.control.throttle = 0
        time.sleep(2)
        break
    time.sleep(0.01)
print ('Meco')
vessel.control.activate_next_stage()
vessel.control.rcs = True
vessel.control.forward = 1
time.sleep(2)
for engine in vessel.parts.engines:
    if not engine.active:
        print ('There is no active engine, checking Propellant condition')
        for engine in engine.part.modules:
            if engine.has_field('Propellant'):
                if engine.get_field('Propellant') == 'Very Stable':
                    print ('Engine is ready')
vessel.control.forward = 0
vessel.control.throttle = 1
vessel.control.activate_next_stage()
while vessel.thrust < vessel.available_thrust:
    time.sleep(0.01)

a,b,c,t = peg(vessel,target_altitude,0,0,200)
fairing_jettison = False
while True:
    a,b,c,t = peg(vessel,target_altitude,a,b,t)
    a,b,c,t1 = peg(vessel,target_altitude,a,b,t)
    if np.absolute(t1-t)/t < 0.01:
        pitch = asind(a + b*0.1 + c)
        vessel.auto_pilot.target_pitch = pitch

    if vessel.flight(orbref).mean_altitude > vessel.orbit.body.atmosphere_depth and not fairing_jettison:
        for part in vessel.parts.all:
            for module in part.modules:
                if module.has_event('Jettison'):
                    module.trigger_event('Jettison')
                    fairing_jettison = True

    if np.absolute(vessel.orbit.apoapsis - vessel.orbit.periapsis)/vessel.orbit.apoapsis < 0.001:
        vessel.control.throttle = 0
        break
    time.sleep(0.01)
print ('Mission Success')
conn.close()
