######################################################################
# fov.jl
######################################################################

type FOVn <: Sensor
	region_probs::Vector{NTuple{2,Float64}}

	# for discretized measurements
	num_bins::Int
	bin_range::UnitRange{Int64}

	FOVn(region_probs::Vector{NTuple{2,Float64}})= new(region_probs, 2, 0:1)
end


# 1 means it is in the field of view
# 0 means it is not
function observe(m::SearchDomain, s::FOVn, p::Pose)

	# ensure bearing is reflected across
    rel_bearing = fit_180(p[3] - noisy_bearing(p, m.theta))
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


# o technically has to be a float, but we only have two obs:
#  1 = in field of view
#  0 = not in field of view
function O(s::FOVn, theta::LocTuple, p::Pose, o::Float64)
    println("this method is being called")

	# determine relative bearing and fix it in 0 to 180
	rel_bearing = fit_180(p[3] - true_bearing(p, theta))
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


function O(s::FOVn, theta::LocTuple, xp::Pose, o::ObsBin)
	# determine relative bearing and fix it in 0 to 180
	rel_bearing = fit_180(xp[3] - noisy_bearing(xp, theta))
	#rel_bearing = fit_180(xp[3] - true_bearing(xp, theta))
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

obs2bin(o::Float64, s::FOVn) = round(Int, o)

# adds some zero-mean gaussian noise to true_bearing
# noise is estimated from cell_size and error in position
function noisy_bearing(p::Pose, theta::LocTuple)

    dx = (p[1] - theta[1])
    dy = (p[2] - theta[2])
    d = sqrt(dx*dx + dy*dy)

    a = 107.462
    b = -1.01731
    bearing_sigma = min(90.0, a * d^b)

    # measured bearing
    mb = true_bearing(p, theta) + bearing_sigma * randn()

    return mb
end
