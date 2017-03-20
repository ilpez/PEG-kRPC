import krpc
import numpy as np
import time
import peg

conn = krpc.connect(name='Powered Explicit Guidance')
vessel = conn.space_center.active_vessel

orbref = vessel.orbit.body.non_rotating_reference_frame
surfref = vessel.orbit.body.reference_frame

g0 = 9.80655
mu = vessel.orbit.body.gravitational_parameter

target_apoapsis = 200
target_periapsis = 200
target_inclination = 51.65
target_lan = 240

(target_velocity,
 target_radius,
 azimuth,
 launch_time) = peg.target_parameter(vessel,
                                     target_apoapsis,
                                     target_periapsis,
                                     target_inclination,
                                     target_lan,
                                     -1)

game_launch_time = conn.space_center.ut + launch_time
conn.space_center.warp_to(game_launch_time - 10)

while (conn.space_center.ut - game_launch_time) < 0:
    print('Time to launch %f' % (conn.space_center.ut - game_launch_time))
    time.sleep(1)

vessel.control.throttle = 1
for engine in vessel.parts.engines:
    if not engine.active:
        print('There is no active engine, checking Propellant condition')
        for engine in engine.part.modules:
            if engine.has_field('Propellant'):
                if engine.get_field('Propellant') == 'Very Stable':
                    print('Engine is ready')
                    vessel.control.activate_next_stage()
while vessel.thrust < vessel.max_thrust:
    time.sleep(0.01)

vessel.auto_pilot.engage()
vessel.auto_pilot.target_heading = 0
vessel.auto_pilot.target_pitch = 90
vessel.control.activate_next_stage()

print('Proceeding Launch..')

while vessel.flight(surfref).speed < 30:
    time.sleep(0.1)

print('Clear from launch tower..')
print('Begin Pitch and Roll Program..')
vessel.auto_pilot.target_heading = azimuth
vessel.auto_pilot.target_roll = azimuth
while True:
    if vessel.auto_pilot.target_roll > 0:
        vessel.auto_pilot.target_roll -= 0.01
    else:
        vessel.auto_pilot.target_roll = 0
    pitch = peg.atand(1000/vessel.flight(surfref).speed)
    vessel.auto_pilot.target_pitch = pitch
    if vessel.flight(surfref).speed > 2700 or vessel.available_thrust == 0:
        vessel.control.throttle = 0
        time.sleep(2)
        break
    time.sleep(0.01)
print('Meco')
vessel.control.activate_next_stage()
vessel.control.rcs = True
vessel.control.forward = 1
time.sleep(2)
for engine in vessel.parts.engines:
    if not engine.active:
        print('There is no active engine, checking Propellant condition')
        for engine in engine.part.modules:
            if engine.has_field('Propellant'):
                if engine.get_field('Propellant') == 'Very Stable':
                    print('Engine is ready')
                    vessel.control.forward = 0
                    vessel.control.throttle = 1
                    vessel.control.activate_next_stage()

while vessel.thrust < vessel.max_thrust:
    time.sleep(0.01)
vessel.auto_pilot.target_roll = 0
t_call = time.time()
delta_t = time.time() - t_call
a, b, c, t = peg.peg(delta_t,
                     vessel,
                     target_velocity,
                     target_radius,
                     0,
                     0,
                     200)
fairing_jettison = False
while True:
    delta_t = time.time() - t_call
    t_call = time.time()
    a, b, c, t = peg.peg(delta_t,
                         vessel,
                         target_velocity,
                         target_radius,
                         a,
                         b,
                         t)
    a, b, c, t1 = peg.peg(delta_t,
                          vessel,
                          target_velocity,
                          target_radius,
                          a,
                          b,
                          t)
    if np.absolute(t1-t)/t < 0.01:
        pitch = peg.asind(a + b*delta_t + c)
        vessel.auto_pilot.target_pitch = pitch

    if (vessel.flight(orbref).mean_altitude >
            vessel.orbit.body.atmosphere_depth and not fairing_jettison):
        for part in vessel.parts.all:
            for module in part.modules:
                if module.has_event('Jettison'):
                    module.trigger_event('Jettison')
                    fairing_jettison = True

    if t1 < 0.01:
        vessel.control.throttle = 0
        vessel.control.rcs = False
        break
    time.sleep(0.01)
print('Mission Success')
conn.close()
