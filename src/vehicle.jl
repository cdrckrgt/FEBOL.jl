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

	Vehicle(x::Float64, y::Float64, ns::Float64) = new(x,y,ns)
	Vehicle(x::Float64, y::Float64) = new(x,y,10.)
end
