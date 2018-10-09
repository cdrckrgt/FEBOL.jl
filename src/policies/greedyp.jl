######################################################################
# greedyp.jl
#
# like greedy.jl, but includes penalty for being too close to target
######################################################################
mutable struct GreedyP <: Policy
    actions::Vector{Action}
    lambda::Float64
    d::Float64
end
function GreedyP(max_step::Real, n::Int, headings=0; stay=true)
    GreedyP(make_action_list(max_step, n, headings, stay=stay), 10.0, 15.0)
end
function GreedyP(x::Vehicle, n::Int, headings=0; stay=true)
    GreedyP(make_action_list(x.max_step, n, headings, stay=stay), 10.0,15.0)
end



# loop over all actions.
# The one with smallest expected entropy is best
# We know this depends on sensor
function action(m::SearchDomain, x::Vehicle, o, f::AbstractFilter, p::GreedyP)
    best_score = -Inf
    best_a = (0.0, 0.0, 0.0)
    best_mi = 0.0
    best_p = 0.0

    # propagate target dynamics forward
    f2 = deepcopy(f)
    predict!(f2)

    for a in p.actions
        # find out where a will take you
        xp = new_pose(m, x, a)
        # compute best mutual information
        mi = mutual_information(f2, xp)

        # compute penalty for being too close
        penalty = p.lambda * nmac_penalty(f2, xp, p.d)

        score = mi - penalty

        if score > best_score
            best_mi = mi
            best_p = penalty
            best_score = score
            best_a = a
        end
    end

    #println("best_mi = ", best_mi, ", best_p = ", best_p)
    #c = covariance(f2)
    #ev = eigvals(c)
    #e1 = ev[1]
    #e2 = ev[2]
    #println("c = ", covariance(f))
    #println("e1 = ", e1, ", e2 = ", e2)
    #println("ce = ", cheap_entropy(f, 200.0, 40))
    return best_a
end

function nmac_penalty(pf::PF, xp::Pose, d::Float64)
    nmac_penalty(pf._b, xp, d, pf.n)
end
function nmac_penalty(b::ParticleCollection, xp::Pose, d::Float64, n::Int)
    penalty = 0.0
    for i = 1:n
        tp = particle(b,i)
        dx = tp[1] - xp[1]
        dy = tp[2] - xp[2]

        if dx*dx + dy*dy < d*d
            penalty += 1.0
        end
    end
    return penalty / n
end
