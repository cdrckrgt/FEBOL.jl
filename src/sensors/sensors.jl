######################################################################
# sensors.jl
######################################################################

abstract type Sensor end

# Returns a possible observation to be received by sensor s from pose p
#  assuming the target is located according to m
function observe(m::SearchDomain, s::Sensor, p::Pose)
    error("`observe` not defined for this sensor.")
end

# Returns probability of measuring o with sensor s if vehicle pose is p
#  and target location is theta
function O(s::Sensor, theta::LocTuple, p::Pose, o)
    error("`O` (density version) not defined for this sensor.")
end

include("bearing.jl")
include("binned_bearing.jl")
include("range.jl")
include("diromni.jl")
include("binned_diromni.jl")
include("fov.jl")
include("fovn.jl")
include("fov3.jl")
