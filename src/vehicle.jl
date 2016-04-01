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

	Vehicle(x::Real,y::Real,ns::Real,ms::Real) = new(float(x),float(y),float(ns),float(ms))
	Vehicle(x::Real,y::Real,ns::Real) = new(float(x),float(y),float(ns),2.0)
	Vehicle(x::Real, y::Real) = new(float(x), float(y), 15.0, 2.0)
end

function new_location(m::SearchDomain, x::Vehicle, a::Action)
	new_x = max(min(x.x + a[1], m.length), 0.0)
	new_y = max(min(x.y + a[2], m.length), 0.0)
	return new_x, new_y
end

function act!(m::SearchDomain, x::Vehicle, a::Action)
	x.x, x.y = new_location(m, x, a)
end
