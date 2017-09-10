######################################################################
# diromni.jl
######################################################################

type DirOmni <: Sensor
	means::Vector{Float64}
	stds::Vector{Float64}

	function DirOmni(file::AbstractString)
		means = vec(readcsv(file)[:,2])
		stds = 2*ones(360)
		return new(means, stds)
	end
end


# returns a density
function observe(m::SearchDomain, s::DirOmni, p::Pose)
	# determine the relative bearing
	rel_bearing = p[3] - true_bearing(p, m.theta)
	if rel_bearing < 0.0
		rel_bearing += 360.0
	end
	rel_int = round(Int, rel_bearing, RoundDown) + 1

	return s.means[rel_int] + s.stds[rel_int]*randn()
end


function O(s::DirOmni, theta::LocTuple, p::Pose, o::Float64)

	# calculate the relative int
	rel_bearing = p[3] - true_bearing(p, theta)
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


#function O(x::Vehicle, s::DirOmni, xp::Pose, theta::LocTuple, o::ObsBin, df::DF)
function O(s::DirOmni, theta::LocTuple, xp::Pose, o::ObsBin)
	#rel_bearing = x.heading - true_bearing(xp, theta)
	rel_bearing = xp[3] - true_bearing(xp, theta)
	if rel_bearing < 0.0
		rel_bearing += 360.0
	end
	rel_int = round(Int, rel_bearing, RoundDown) + 1

	low_val = floor(o)
	high_val = low_val + 1
	d = Normal(s.means[rel_int], s.stds[rel_int])
	p = cdf(d, high_val) - cdf(d, low_val)
	return p
end

# here, num_bins isn't too important; we just bin to nearest integer
obs2bin(o::Float64, s::DirOmni) = round(Int, o, RoundDown)
