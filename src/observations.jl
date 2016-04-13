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
"""
`observe(m::SearchDomain, x::Vehicle)`

Sample an observation. Returns a float between 0 and 360.
"""
function observe(m::SearchDomain, x::Vehicle)
	observe(m::SearchDomain, x, x.sensor)
end

function observe(m::SearchDomain, x::Vehicle, s::BearingOnly)
	truth = true_bearing(x, m.theta)
	noise = s.noise_sigma * randn()
	return mod(truth + noise, 360.0)
end

function observe(m::SearchDomain, x::Vehicle, s::DirOmni)
	rel_bearing = x.heading - true_bearing(x, m.theta)
	# round to int first
	noise = 32
	s.means[rel_bearing]
end

# Fits an angle into -180 to 180
# Assume the angle is within -360 to 360
function fit_180(angle::Float64)
	if angle > 180.0
		#angle = 360.0 - angle   # omfg this is how i did it before. wrong.
		angle -= 360.0
	elseif angle < -180.0
		angle += 360.0
	end
	return angle
end

# doesn't actually return a probability. It returns a density
# TODO: I think we need to make this generic
function O(x::Vehicle, theta::LocTuple, o::Float64)

	# Calculate true bearing, and find distance to bin edges
	ang_deg = true_bearing(x, theta)
	#rel_start, rel_end = rel_bin_edges(ang_deg, o, df)
	o_diff = fit_180(o - ang_deg)

	# now look at probability
	d = Normal(0, x.sensor.noise_sigma)
	#p = cdf(d, rel_end) - cdf(d, rel_start)
	p = pdf(d, o_diff)
	return p
end
