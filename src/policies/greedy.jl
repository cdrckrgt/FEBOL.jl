######################################################################
# greedy.jl
#
# Defines a greedy, info-theoretic policy
# Currently only considers a small set of the possible actions
######################################################################
struct GreedyPolicy <: Policy
    actions::Vector{Action}

    GreedyPolicy() = new()

    function GreedyPolicy(max_step::Real, n::Int; stay=true)
        return new( make_action_list(max_step, n, 0) )
    end
    function GreedyPolicy(max_step::Real, n::Int, headings; stay=true)
        return new( make_action_list(max_step, n, headings) )
    end
    function GreedyPolicy(x::Vehicle, n::Int, headings; stay=true)
        return new( make_action_list(x.max_step, n, headings) )
    end
end



# loop over all actions.
# The one with smallest expected entropy is best
# We know this depends on sensor
function action(m::SearchDomain, x::Vehicle, o, f::DF, p::GreedyPolicy)
    best_mi = -Inf
    best_a = (0.0, 0.0, 0.0)
    for a in p.actions
        # find out where a will take you
        xp = new_pose(m, x, a)
        # compute best mutual information
        mi = mutual_information(f, xp)
        #println("a = ", a)
        #println("\tmi = ", mi)
        if mi > best_mi
            best_mi = mi
            best_a = a
        end
    end
    return best_a
end
