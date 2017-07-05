######################################################################
# bearing.jl
#
# bearing only sensing modality
######################################################################

type BearingOnly <: Sensor
	noise_sigma::Float64		# noise std deviation (degrees)
end


function observe(m::SearchDomain, s::BearingOnly, p::Pose)
	truth = true_bearing(p, m.theta)
	noise = s.noise_sigma * randn()
	return mod(truth + noise, 360.0)
end


# Called by PF
# doesn't actually return a probability. It returns a density
#function O(x::Vehicle, s::BearingOnly, theta::LocTuple, o::Float64)
# TODO: give theta a type
function O(bo::BearingOnly, theta, p::Pose, o::Float64)

	# Calculate true bearing, and find distance to bin edges
	ang_deg = true_bearing(p, theta)
	#rel_start, rel_end = rel_bin_edges(ang_deg, o, df)
	o_diff = fit_180(o - ang_deg)

	# now look at probability
	d = Normal(0, bo.noise_sigma)
	#p = cdf(d, rel_end) - cdf(d, rel_start)
	p = pdf(d, o_diff)
	return p
end
