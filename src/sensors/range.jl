######################################################################
# range.jl
#
# range only sensing
######################################################################

type RangeOnly <: Sensor
	noise_sigma::Float64		# noise std deviation

	# for discretized measurements
	num_bins::Int
	bin_range::UnitRange{Int64}

	function RangeOnly(noise_sigma::Real, num_bins::Int)
		new( float(noise_sigma), num_bins, 0:(num_bins-1) )
	end
	RangeOnly(noise_sigma::Real) = RangeOnly(noise_sigma, 1000)
end


function observe(m::SearchDomain, ro::RangeOnly, p::Pose)
	dx = (p[1] - m.theta[1])
	dy = (p[2] - m.theta[2])
	return sqrt(dx*dx + dy*dy) + ro.noise_sigma*randn()
end

# just returns a density. used in particle filtering
function O(ro::RangeOnly, theta::LocTuple, p::Pose, o::Float64)
	dx = (p[1] - theta[1])
	dy = (p[2] - theta[2])
	o_diff = sqrt(dx*dx + dy*dy) - o

	d = Normal(0, ro.noise_sigma)
	p = pdf(d, o_diff)
	return p
end

# just returns a density. used in particle filtering
# TODO: make this be correct
function O(ro::RangeOnly, theta::LocTuple, p::Pose, o::Int)
	dx = (p[1] - theta[1])
	dy = (p[2] - theta[2])
	o_diff = sqrt(dx*dx + dy*dy) - o

	d = Normal(0, ro.noise_sigma)
	p = pdf(d, o_diff)
	return p
end

# here, num_bins isn't important; we just bin to nearest integer
obs2bin(o::Float64, s::RangeOnly) = round(Int, o, RoundDown)
