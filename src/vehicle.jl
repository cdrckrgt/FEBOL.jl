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
	noise_sigma::Float64
	max_step::Float64

	Vehicle(x::Float64, y::Float64,ns::Float64,mv::Float64) = new(x,y,ns,mv)
	Vehicle(x::Float64, y::Float64, ns::Float64) = new(x,y,ns,2.0)
	Vehicle(x::Float64, y::Float64) = new(x,y,10.0,2.0)
end

function new_location(m::SearchDomain, x::Vehicle, a::Action)
	new_x = max(min(x.x + a[1], m.length), 0.0)
	new_y = max(min(x.y + a[2], m.length), 0.0)
	return new_x, new_y
end

function act!(m::SearchDomain, x::Vehicle, a::Action)
	x.x, x.y = new_location(m, x, a)
end
