######################################################################
# fov3.jl
# Like FOV, but has 3 groups instead of 2
######################################################################

type FOV3 <: Sensor
	region_probs::Vector{NTuple{2,Float64}}

	# for discretized measurements
	num_bins::Int
	bin_range::UnitRange{Int64}

	#FOV3(region_probs::Vector{NTuple{2,Float64}}) = new(region_probs, 2, 0:1)

    FOV3() = new([(60.0,.9), (120.0,0.9), (180.0,0.9)], 3, 1:3)
end


# 1 means it is in the field of view
# 0 means it is not
function observe(m::SearchDomain, s::FOV3, p::Pose)

	# ensure bearing is reflected across
	rel_bearing = fit_180(p[3] - true_bearing(p, m.theta))
	if rel_bearing < 0.0
		rel_bearing = -1.0 * rel_bearing
	end

    # find the region in which this rel bearing lies
	prob_in_view = 0.0
	n = length(s.region_probs)
    expected_obs = 1
	for i = 1:n
		temp_angle, temp_prob = s.region_probs[i]
		if rel_bearing  <= temp_angle
            expected_obs = i
			prob_in_view = temp_prob
			break
		end
	end

    # TODO: make this less inefficient
    obs_vals = [1,2,3]
    o_probs  = 0.5*(1.0-0.9) * ones(3)

    o_probs[expected_obs] = 0.9 #prob_in_view

    r = rand()
    o = 1
    if r < o_probs[1]
        o = obs_vals[1]
    elseif r < o_probs[2]
        o = obs_vals[2]
    else
        o = obs_vals[3]
    end

    return o * 1.0
end


# o technically has to be a float, but we only have two obs:
#  1 = in field of view
#  0 = not in field of view
function O(s::FOV3, theta::LocTuple, p::Pose, o::Float64)

	# determine relative bearing and fix it in 0 to 180
	rel_bearing = fit_180(p[3] - true_bearing(p, theta))
	if rel_bearing < 0.0
		rel_bearing = -1.0 * rel_bearing
	end

	prob_in_view = 0.0
	n = length(s.region_probs)
    expected_obs = 1
	for i = 1:n
		temp_angle, temp_prob = s.region_probs[i]
		if rel_bearing  <= temp_angle
			prob_in_view = temp_prob
            expected_obs = i * 1.0
			break
		end
	end

    ret_val = (o == expected_obs) ? 0.9 : 0.05

	return ret_val
end


# I don't really get why this is here...
#function O(x::Vehicle,s::FOV3,xp::Pose, theta::LocTuple, o::ObsBin, df::DF)
function O(s::FOV3, theta::LocTuple, xp::Pose, o::ObsBin)
	# determine relative bearing and fix it in 0 to 180
	rel_bearing = fit_180(xp[3] - true_bearing(xp, theta))
	if rel_bearing < 0.0
		rel_bearing = -1.0 * rel_bearing
	end

	prob_in_view = 0.0
	n = length(s.region_probs)
    expected_obs = 1
	for i = 1:n
		temp_angle, temp_prob = s.region_probs[i]
		if rel_bearing  <= temp_angle
			prob_in_view = temp_prob
            expected_obs = i * 1.0
			break
		end
	end

    ret_val = (o == expected_obs) ? 0.9 : 0.05
	return ret_val
end

obs2bin(o::Float64, s::FOV3) = round(Int, o)
