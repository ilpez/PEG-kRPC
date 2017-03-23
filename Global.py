import krpc

conn = krpc.connect(name='Launch to orbit')
space_center = conn.space_center
vessel = space_center.active_vessel
g0 = vessel.orbit.body.surface_gravity
mu = vessel.orbit.body.gravitational_parameter
