######################################################################
# range.jl
#
# range only sensing
######################################################################

type RangeOnly <: Sensor
	noise_sigma::Float64		# noise std deviation
end

function observe(m::SearchDomain, ro::RangeOnly, p::Pose)
	dx = (p[1] - m.theta[1])
	dy = (p[2] - m.theta[2])
	return sqrt(dx*dx + dy*dy) + ro.noise_sigma*randn()
end

# just returns a density. used in particle filtering
function O(ro::RangeOnly, theta, p::Pose, o::Float64)
	dx = (p[1] - theta[1])
	dy = (p[2] - theta[2])
	o_diff = sqrt(dx*dx + dy*dy) - o

	d = Normal(0, x.sensor.noise_sigma)
	p = pdf(d, o_diff)
	return p
end
