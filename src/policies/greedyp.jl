######################################################################
# greedyp.jl
#
# like greedy.jl, but includes penalty for being too close to target
######################################################################
mutable struct GreedyP <: Policy
    actions::Vector{Action}
    lambda::Float64
    d::Float64

    GreedyP() = new()

    function GreedyP(max_step::Real, n::Int, headings=0; stay=true)
        return new( make_action_list(max_step, n, headings, stay=stay) , 10.0, 15.0)
    end
    function GreedyP(x::Vehicle, n::Int, headings=0; stay=true)
        return new( make_action_list(x.max_step, n, headings, stay=stay), 10.0, 15.0)
    end
end



# loop over all actions.
# The one with smallest expected entropy is best
# We know this depends on sensor
function action(m::SearchDomain, x::Vehicle, o, f::AbstractFilter, p::GreedyP)
    best_score = -Inf
    best_a = (0.0, 0.0, 0.0)
    for a in p.actions
        # find out where a will take you
        xp = new_pose(m, x, a)
        # compute best mutual information
        mi = mutual_information(f, xp)

        # compute penalty for being too close
        penalty = p.lambda * get_penalty(f, xp, p.d)

        score = mi + penalty

        if score > best_score
            best_score = score
            best_a = a
        end
    end
    return best_a
end

function get_penalty(pf::PF, xp::Pose, d::Float64)
    penalty = 0.0
    for i = 1:pf.n
        tp = particle(pf,i)
        dx = tp[1] - xp[1]
        dy = tp[2] - xp[2]

        if dx*dx + dy*dy < d*d
            penalty -= weight(pf, i)
        end
    end
    return penalty / weight_sum(pf)
end
