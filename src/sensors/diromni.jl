######################################################################
# diromni.jl
######################################################################

struct DirOmni <: Sensor
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


function O(s::DirOmni, theta::LocTuple, p::Pose, o)

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
