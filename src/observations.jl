######################################################################
# observations.jl
#
# Contains everything needed for the observation function.
######################################################################

# Find the true angle between UAV and jammer.
#
# Parameters:
#  xp - location of vehicle
#  theta - location of jammer
#
# Returns true angle, measured from north, in degrees.
function true_bearing(xp::LocTuple, theta::Vector{Float64})
	return true_bearing(xp, (theta[1], theta[2]))
end
true_bearing(x::Vehicle, theta) = true_bearing( (x.x, x.y), theta)
function true_bearing(xp::LocTuple, theta::LocTuple)
	xr = theta[1] - xp[1]
	yr = theta[2] - xp[2]
	return mod(rad2deg(atan2(xr,yr)), 360)
end

# Steps:
#  1. compute the true bearing
#  2. sample the error
function observe(m::SearchDomain, x::Vehicle)
	truth = true_bearing(x, m.theta)
	noise = x.noise_sigma * randn()
	return mod(truth + noise, 360.0)
end
