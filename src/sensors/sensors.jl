######################################################################
# sensors.jl
######################################################################

abstract type Sensor end

include("bearing.jl")
include("range.jl")
include("diromni.jl")
include("fov.jl")

function observe(m::SearchDomain, s::Sensor, p::Pose)
	error("`observe` not defined for this sensor.")
end

# Required for particle filter
# does not require probability, returns density
function O(s::Sensor, theta::LocTuple, p::Pose, o::Float64)
	error("`O` (density version) not defined for this sensor.")
end
