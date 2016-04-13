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

	function Vehicle(x::Real,y::Real,h::Real,ms::Real,s::Sensor)
		return new(float(x),float(y),float(h),float(ms),s)
	end
	#Vehicle(x::Real,y::Real,ns::Real) = new(float(x),float(y),float(ns),2.0)

	function Vehicle(x::Real, y::Real)
		return new( float(x), float(y), 0.0, 2.0, BearingOnly(10.0) )
	end
end

function new_location(m::SearchDomain, x::Vehicle, a::Action)
	new_x = max(min(x.x + a[1], m.length), 0.0)
	new_y = max(min(x.y + a[2], m.length), 0.0)
	return new_x, new_y
end

"""
`act!(m::SearchDomain, x::Vehicle, a::Action)`
"""
function act!(m::SearchDomain, x::Vehicle, a::Action)
	x.x, x.y = new_location(m, x, a)
end
