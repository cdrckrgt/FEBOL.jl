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

"""
`observe(m::SearchDomain, x::Vehicle)`

Sample an observation. Returns a float between 0 and 360.
"""
function observe(m::SearchDomain, x::Vehicle)
	observe(m, x, x.sensor)
end

function observe(m::SearchDomain, x::Vehicle, s::BearingOnly)
	truth = true_bearing(x, m.theta)
	noise = s.noise_sigma * randn()
	return mod(truth + noise, 360.0)
end

function observe(m::SearchDomain, x::Vehicle, s::DirOmni)
	# determine the relative bearing
	rel_bearing = x.heading - true_bearing(x, m.theta)
	if rel_bearing < 0.0
		rel_bearing += 360.0
	end
	rel_int = round(Int, rel_bearing, RoundDown) + 1

	return s.means[rel_int] + s.stds[rel_int]*randn()
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



######################################################################
# O(x, theta, o)
# Required for pf
######################################################################

function O(x::Vehicle, theta::LocTuple, o::Float64)
	return O(x, x.sensor, theta, o)
end
# Called by PF
# doesn't actually return a probability. It returns a density
function O(x::Vehicle, s::BearingOnly, theta::LocTuple, o::Float64)

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

function O(x::Vehicle, s::DirOmni, theta::LocTuple, o::Float64)

	rel_bearing = x.heading - true_bearing(x, theta)
	if rel_bearing < 0.0
		rel_bearing += 360.0
	end
	rel_int = round(Int, rel_bearing, RoundDown) + 1

	# Calculate true bearing, and find distance to bin edges
	ang_deg = true_bearing(x, theta)
	#rel_start, rel_end = rel_bin_edges(ang_deg, o, df)
	#o_diff = fit_180(o - ang_deg)
	o_diff = o - x.sensor.means[rel_int]

	# now look at probability
	d = Normal(0, x.sensor.stds[rel_int])
	#p = cdf(d, rel_end) - cdf(d, rel_start)
	p = pdf(d, o_diff)
	return p
end
