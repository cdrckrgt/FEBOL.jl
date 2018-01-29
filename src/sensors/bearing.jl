######################################################################
# bearing.jl
#
# bearing only sensing modality
######################################################################

struct BearingOnly <: Sensor
    noise_sigma::Float64		# noise std deviation (degrees)

    BearingOnly(ns::Real) = new(ns)
    BearingOnly() = new(10.0)
end


function observe(m::SearchDomain, s::BearingOnly, p::Pose)
    truth = true_bearing(p, m.theta)
    noise = s.noise_sigma * randn()
    return mod(truth + noise, 360.0)
end


# Called by PF
# doesn't actually return a probability. It returns a density
#function O(x::Vehicle, s::BearingOnly, theta::LocTuple, o::Float64)
# TODO: give theta a type (probably LocTuple?)
function O(bo::BearingOnly, theta, p::Pose, o)

    # Calculate true bearing, and find distance to bin edges
    ang_deg = true_bearing(p, theta)
    o_diff = fit_180(o - ang_deg)

    # now look at probability
    d = Normal(0, bo.noise_sigma)
    p = pdf(d, o_diff)
    return p
end
