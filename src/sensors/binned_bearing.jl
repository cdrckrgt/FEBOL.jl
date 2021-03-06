######################################################################
# binned_bearing.jl
#
# like bearing only, but observations are discretized into bins
######################################################################

export BinnedBearing
struct BinnedBearing <: Sensor
    noise_sigma::Float64		# noise std deviation (degrees)

    # for discretized measurements
    num_bins::Int
    bin_range::UnitRange{Int64}

    # TODO: technically must ensure noise_sigma is positive
    function BinnedBearing(noise_sigma::Real, num_bins::Int)
        new( float(noise_sigma), num_bins, 0:(num_bins-1))
    end
    BinnedBearing(noise_sigma::Real) = BinnedBearing(noise_sigma, 36)
    BinnedBearing() = BinnedBearing(10.0)
end


function observe(m::SearchDomain, s::BinnedBearing, p::Pose)
    truth = true_bearing(p, m.theta)
    noise = s.noise_sigma * randn()
    return obs2bin(mod(truth + noise, 360.0), s)
end


function O(bo::BinnedBearing, theta::LocTuple, xp::Pose, o::Int64)

    # Calculate true bearing, and find distance to bin edges
    ang_deg = true_bearing(xp, theta)
    rel_start, rel_end = rel_bin_edges(ang_deg, o, bo)

    # now look at probability
    sig = bo.noise_sigma
    p = my_cdf(sig, rel_end) - my_cdf(sig, rel_start)
    return p
end

# Find the relative offset
# TODO: must account for different discretizations
#function rel_bin_edges(bearing_deg, o::ObsBin, df::DF)
function rel_bin_edges(bearing_deg, o::ObsBin, bo::BinnedBearing)

    # calculate start, end degrees of bin
    start_deg, end_deg = bin2deg(o, bo)

    # compute relative distance to true bearing
    rel_start = fit_180(bearing_deg - start_deg)
    rel_end = fit_180(bearing_deg - end_deg)

    #rel_start = min(abs(rel_start), abs(rel_end))
    #rel_end = rel_start + 360.0 / df.num_bins

    # Make sure start is further left on number line
    if rel_end < rel_start
        temp = rel_start
        rel_start = rel_end
        rel_end = temp
    end

    # If we straddle the wrong point
    # Say df.num_bins = 10, and rel_start = -175, rel_end = 175
    # rel_end - rel_start would be 350 degrees, but this should be 10
    # so set rel_start to 175 and rel_end to 185
    # TODO: I'm pretty sure this doesn't work for df.num_bins = 2
    if (rel_end - rel_start) - 1e-3 > (360.0/bo.num_bins)
        rel_start = rel_end
        rel_end += 360.0/bo.num_bins
    end

    return rel_start, rel_end
end

# returns (start_deg, end_deg) integer tuple
function bin2deg(bin_deg::Int, bo::BinnedBearing)
    full_bin = 360.0 / bo.num_bins
    half_bin = full_bin / 2.0
    if bin_deg == 0
        start_val = -half_bin
        end_val = half_bin
    else
        start_val = full_bin * bin_deg - half_bin
        end_val  = full_bin * bin_deg + half_bin
    end
    return start_val, end_val
end

# 355 - 4.9999 = 0
# 5 - 14.9999 = 1
# 15 - 24.999 = 2
function obs2bin(o::Float64, s::BinnedBearing)
    full_bin = 360.0 / s.num_bins
    half_bin = full_bin / 2.0

    ob = round( Int, div((o + half_bin), full_bin) )
    if ob == s.num_bins
        ob = 0
    end
    return ob
end
