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
function true_bearing(xp::Pose, theta::Vector{Float64})
	return true_bearing((xp[1], xp[2]), (theta[1], theta[2]))
end
true_bearing(x::Vehicle, theta) = true_bearing( (x.x, x.y), theta)
true_bearing(p::Pose, theta::LocTuple) = true_bearing( (p[1], p[2]), theta)
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

# 1 means it is in the field of view
# 0 means it is not
function observe(m::SearchDomain, x::Vehicle, s::FOV)

	# ensure bearing is reflected across
	rel_bearing = fit_180(x.heading - true_bearing(x, m.theta))
	if rel_bearing < 0.0
		rel_bearing = -1.0 * rel_bearing
	end

	prob_in_view = 0.0
	n = length(s.region_probs)
	for i = 1:n
		temp_angle, temp_prob = s.region_probs[i]
		if rel_bearing  <= temp_angle
			prob_in_view = temp_prob
			break
		end
	end

	o = (rand() < prob_in_view) ? 1.0 : 0.0
	return o
end


function angle_mean(v)
	n = length(v)
	sin_sum = 0.0
	cos_sum = 0.0
	for i = 1:n
		sin_sum += sin(v[i])
		cos_sum += cos(v[i])
	end
	return mod(rad2deg(atan2(sin_sum, cos_sum)), 360.)
end

function angle_mean(v,w)
	n = length(v)
	sin_sum = 0.0
	cos_sum = 0.0
	for i = 1:n
		sin_sum += w[i] * sin(v[i])
		cos_sum += w[i] * cos(v[i])
	end
	return mod(rad2deg(atan2(sin_sum, cos_sum)), 360.)
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

	# calculate the relative int
	rel_bearing = x.heading - true_bearing(x, theta)
	if rel_bearing < 0.0
		rel_bearing += 360.0
	end
	rel_int = round(Int, rel_bearing, RoundDown) + 1

	# Calculate expected measurement
	o_diff = o - s.means[rel_int]

	# now look at probability
	d = Normal(0, s.stds[rel_int])
	#p = cdf(d, rel_end) - cdf(d, rel_start)
	p = pdf(d, o_diff)
	return p
end

# o technically has to be a float, but we only have two obs:
#  1 = in field of view
#  0 = not in field of view
function O(x::Vehicle, s::FOV, theta::LocTuple, o::Float64)

	# determine relative bearing and fix it in 0 to 180
	rel_bearing = fit_180(x.heading - true_bearing(x, theta))
	if rel_bearing < 0.0
		rel_bearing = -1.0 * rel_bearing
	end

	prob_in_view = 0.0
	n = length(s.region_probs)
	for i = 1:n
		temp_angle, temp_prob = s.region_probs[i]
		if rel_bearing  <= temp_angle
			prob_in_view = temp_prob
			break
		end
	end

	ret_val = (o == 1.0) ? prob_in_view : (1.0 - prob_in_view)
	return ret_val
end
