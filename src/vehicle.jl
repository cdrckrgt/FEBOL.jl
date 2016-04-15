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

# Does the bounds checking on the action, to see if stays in search domain
# Ensures heading is within [0,360) (other parts of code assume this)
function new_location(m::SearchDomain, x::Vehicle, a::Action)
	new_x = max(min(x.x + a[1], m.length), 0.0)
	new_y = max(min(x.y + a[2], m.length), 0.0)
	new_h = mod(x.heading + a[3], 360.0)
	return new_x, new_y, new_h
end

"""
`act!(m::SearchDomain, x::Vehicle, a::Action)`
"""
function act!(m::SearchDomain, x::Vehicle, a::Action)
	x.x, x.y, x.heading = new_location(m, x, a)
end
