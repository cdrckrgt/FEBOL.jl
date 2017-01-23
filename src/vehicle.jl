######################################################################
# vehicle.jl
# 
######################################################################

# Vehicle is the mobile sensor
# x is east-west position
# y is north-south position
# (0,0) is south west
type Vehicle
	x::Float64
	y::Float64
	heading::Float64
	max_step::Float64
	sensor::Sensor

	function Vehicle(x::Real, y::Real, h::Real, ms::Real, s::Sensor)
		return new(float(x),float(y),float(h),float(ms),s)
	end

	# Default uses BearingOnly
	function Vehicle(x::Real, y::Real)
		return new( float(x), float(y), 0.0, 2.0, BearingOnly(10.0) )
	end
	function Vehicle(x::Real, y::Real, s::Sensor)
		return new( float(x), float(y), 0.0, 2.0, s)
	end

end

function reset!(m::SearchDomain, x::Vehicle)
	x.x = m.length/2.0
	x.y = m.length/2.0
	x.heading = 0.0
end

# Does the bounds checking on the action, to see if stays in search domain
# Ensures heading is within [0,360) (other parts of code assume this)
function new_pose(m::SearchDomain, x::Vehicle, a::Action)
	return new_pose(m, (x.x, x.y, x.heading), a)
end
# TODO: add vehicle limits...
function new_pose(m::SearchDomain, p::Pose, a::Action)
	#new_x = max(min(p[1] + a[1], m.length), 0.0)
	#new_y = max(min(p[2] + a[2], m.length), 0.0)
	new_x = p[1] + a[1]
	new_y = p[2] + a[2]
	new_h = mod(p[3] + a[3], 360.0)
	return new_x, new_y, new_h
end

"""
`act!(m::SearchDomain, x::Vehicle, a::Action)`
"""
function act!(m::SearchDomain, x::Vehicle, a::Action)
	x.x, x.y, x.heading = new_pose(m, x, a)
end
