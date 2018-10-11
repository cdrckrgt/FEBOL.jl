######################################################################
# greedyp2.jl
#
# like greedyp, but uses cheap entropy
######################################################################
export GreedyP2
mutable struct GreedyP2 <: Policy
    actions::Vector{Action}
    lambda::Float64
    d::Float64
end
function GreedyP2(max_step::Real, n::Int, headings=0; stay=true)
    GreedyP2(make_action_list(max_step, n, headings, stay=stay), 10.0, 15.0)
end
function GreedyP2(x::Vehicle, n::Int, headings=0; stay=true)
    GreedyP2(make_action_list(x.max_step, n, headings, stay=stay), 10.0,15.0)
end



# loop over all actions.
# The one with smallest expected entropy is best
# We know this depends on sensor
function action(m::SearchDomain, x::Vehicle, o, f::AbstractFilter, p::GreedyP2)
    best_score = -Inf
    best_a = (0.0, 0.0, 0.0)

    # propagate target dynamics forward
    f2 = deepcopy(f)
    predict!(f2)

    db = zeros(50,50)

    for a in p.actions
        # find out where a will take you
        xp = new_pose(m, x, a)

        # TODO: this only works for FOV
        score = 0.0
        for o = 0:1
            f2 = deepcopy(f)
            update!(f2, xp, o)
            ce = cheap_entropy(f2._b, db, m.length)

            penalty = p.lambda * nmac_penalty(f2, xp, p.d)

            score -= p_obs(f, xp, o) * (ce + penalty)
        end


        if score > best_score
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
