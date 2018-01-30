######################################################################
# binned_diromni.jl
#
# like diromni but discretized
######################################################################

export BinnedDirOmni
mutable struct BinnedDirOmni <: Sensor
    means::Vector{Float64}
    stds::Vector{Float64}

    bin_range::UnitRange{Int64}

    function BinnedDirOmni(file::AbstractString)
        means = vec(readcsv(file)[:,2])
        stds = 2*ones(360)
        bin_range = -30:30
        return new(means, stds, bin_range)
    end
end


function observe(m::SearchDomain, s::BinnedDirOmni, p::Pose)
    # determine the relative bearing
    rel_bearing = p[3] - true_bearing(p, m.theta)
    if rel_bearing < 0.0
        rel_bearing += 360.0
    end
    rel_int = round(Int, rel_bearing, RoundDown) + 1

    float_obs = s.means[rel_int] + s.stds[rel_int]*randn()
    return obs2bin(float_obs, s)
end


function O(s::BinnedDirOmni, theta::LocTuple, xp::Pose, o::Int)
    #rel_bearing = x.heading - true_bearing(xp, theta)
    rel_bearing = xp[3] - true_bearing(xp, theta)
    #println("rel_bearing = ", rel_bearing)
    if rel_bearing < 0.0
        rel_bearing += 360.0
    end
    # had to do this because rel_bearing was sometimes -2.8e-14
    # when adding this to 360.0, julia returned 360.0
    if rel_bearing == 360.0
        rel_bearing -= 1.0
    end
    rel_int = round(Int, rel_bearing, RoundDown) + 1

    low_val = floor(o)
    high_val = low_val + 1
    d = Normal(s.means[rel_int], s.stds[rel_int])
    p = cdf(d, high_val) - cdf(d, low_val)
    return p
end

# here, num_bins isn't too important; we just bin to nearest integer
obs2bin(o::Float64, s::BinnedDirOmni) = round(Int, o, RoundDown)
