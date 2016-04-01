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



function O(m::SearchDomain, xv::Float64, yv::Float64, theta, o::Obs)

	# Calculate true bearing, and find distance to bin edges
	xp = (xv, yv)
	ang_deg = true_bearing(xp, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o)

	# now look at probability
	#p = cdf(m.d, deg2rad(rel_end)) - cdf(m.d, deg2rad(rel_start))
	d = Normal(0, 10.0)
	p = cdf(d, rel_end) - cdf(d, rel_start)
	return p
end


# Fits an angle into -180 to 180
# Assume the angle is within -360 to 360
function fit_180(angle)
	if angle > 180
		angle = 360 - angle
	elseif angle < -180
		angle += 360
	end
	return angle
end
