######################################################################
# fov.jl
#
# field of view sensor
######################################################################

struct FOV <: Sensor
    cone_width::Float64
    alpha::Float64          # mistake rate

    blind_distance::Float64     # distance in which any obs is equal likely
end
function FOV(cone_width::Real, alpha::Float64)
    return FOV(float(cone_width), alpha, 0.0)
end
FOV() = FOV(120.0, 0.1, 0.0)


# 1 means it is in the field of view of front antenna
# 0 means it is not
function observe(theta::TargetTuple, s::FOV, p::Pose)

    # ensure bearing is reflected across
    rel_bearing = fit_180(p[3] - true_bearing(p, theta))
    if rel_bearing < 0.0
        rel_bearing = -1.0 * rel_bearing
    end

    a1 = s.cone_width / 2.0
    a2 = 180.0 - a1

    prob_in_view = 0.0
    if rel_bearing < a1
        prob_in_view = 1.0 - s.alpha
    elseif rel_bearing < a2
        prob_in_view = 0.5
    else
        prob_in_view = s.alpha
    end


    # if we are too close, then prob_in_view = 0.5
    dx = p[1] - theta[1]
    dy = p[2] - theta[2]
    if (dx*dx + dy*dy) < s.blind_distance*s.blind_distance
        prob_in_view = 0.5
    end

    o = (rand() < prob_in_view) ? 1 : 0
    return o
end


# o technically has to be a float, but we only have two obs:
#  1 = in field of view
#  0 = not in field of view
function O(s::FOV, theta::LocTuple, p::Pose, o)

    # determine relative bearing and fix it in 0 to 180
    rel_bearing = fit_180(p[3] - true_bearing(p, theta))
    if rel_bearing < 0.0
        rel_bearing = -1.0 * rel_bearing
    end

    a1 = s.cone_width / 2.0
    a2 = 180.0 - a1

    prob_in_view = 0.0  # probability in view of front antenna

    if rel_bearing < a1
        prob_in_view = 1.0 - s.alpha
    elseif rel_bearing < a2
        prob_in_view = 0.5
    else
        prob_in_view = s.alpha
    end

    # if we are too close, then prob_in_view = 0.5
    dx = p[1] - theta[1]
    dy = p[2] - theta[2]
    if (dx*dx + dy*dy) < s.blind_distance*s.blind_distance
        prob_in_view = 0.5
    end

    # prob that it's one (in view of front antenna)
    ret_val = (o == 1) ? prob_in_view : (1.0 - prob_in_view)

    return ret_val
end


get_prob(s::FOV, theta::LocTuple, xp::Pose, o) = O(s, theta, xp, o)
export get_prob
