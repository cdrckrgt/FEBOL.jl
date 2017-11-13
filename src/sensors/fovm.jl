######################################################################
# fovm.jl
#
# like FOV, but allow for more regions?
######################################################################

type FOVm <: Sensor
	region_probs::Vector{NTuple{2,Float64}}

	# for discretized measurements
	num_bins::Int
	bin_range::UnitRange{Int64}

	FOVm(region_probs::Vector{NTuple{2,Float64}}) = new(region_probs, 2, 0:1)

    function FOVm(cone_width::Real, alpha::Float64)
        a1 = cone_width / 2.0
        a2 = 180.0 - a1
        a3 = 180.0

        region_probs = [(a1,1.0-alpha), (a2, 0.5), (a3, alpha)]

        return new(region_probs, 2, 0:1)
    end
end


# 1 means it is in the field of view
# 0 means it is not
function observe(m::SearchDomain, s::FOVm, p::Pose)

	# ensure bearing is reflected across
	rel_bearing = fit_180(p[3] - true_bearing(p, m.theta))
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
function O(s::FOVm, theta::LocTuple, p::Pose, o::Float64)

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


# I don't really get why this is here...
#function O(x::Vehicle,s::FOV, xp::Pose, theta::LocTuple, o::ObsBin, df::DF)
function O(s::FOVm, theta::LocTuple, xp::Pose, o::ObsBin)
	# determine relative bearing and fix it in 0 to 180
	rel_bearing = fit_180(xp[3] - true_bearing(xp, theta))
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

obs2bin(o::Float64, s::FOVm) = round(Int, o)
