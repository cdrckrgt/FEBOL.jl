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

function O(bo::BearingOnly, theta::TargetTuple, p::Pose, o)

    # Calculate true bearing, and find distance to bin edges
    ang_deg = true_bearing(p, theta)
    o_diff = fit_180(o - ang_deg)

    # now look at probability
    d = Normal(0, bo.noise_sigma)
    p = pdf(d, o_diff)
    return p
end
