######################################################################
# greedy.jl
#
# Defines a greedy, info-theoretic policy
# Currently only considers a small set of the possible actions
######################################################################
type GreedyPolicy <: Policy
	actions::Vector{Action}

    GreedyPolicy() = new()

    function GreedyPolicy(max_step, n, headings; stay=true)
        return new( make_action_list(max_step, n, headings) )
    end


    # Functions below are garbage and should be eliminated
	function GreedyPolicy(x::Vehicle, n::Int)
		return GreedyPolicy(x, x.sensor, n)
	end
	function GreedyPolicy(x::Vehicle, ::BearingOnly, n::Int)
        actions = make_action_list(x.max_step, n, 0)
		return new(n, actions)
	end
	function GreedyPolicy(x::Vehicle, ::RangeOnly, n::Int)
        actions = make_action_list(x.max_step, n, 0)
		return new(n, actions)
	end
	function GreedyPolicy(x::Vehicle, ::DirOmni, n::Int)
        actions = make_action_list(x.max_step, n, [-10,0,10])
		return new(n, actions)
	end

	function GreedyPolicy(x::Vehicle, ::FOV, n::Int)
		actions = make_action_list(x.max_step, n, [-10,0,10])
		return new(n, actions)
	end
	function GreedyPolicy(x::Vehicle, ::FOV3, n::Int)
        actions = make_action_list(x.max_step, n, [-10,0,10])
		return new(n, actions)
	end
end



# loop over all actions.
# The one with smallest expected entropy is best
# We know this depends on sensor
function action(m::SearchDomain, x::Vehicle, o::Float64, f::DF, p::GreedyPolicy)
	best_mi = -Inf
	best_a = (0.0, 0.0, 0.0)
	for a in p.actions
		# find out where a will take you
		xp = new_pose(m, x, a)
		# compute best mutual information
		mi = mutual_information(m, x, f, xp)
		#println("a = ", a)
		#println("\tmi = ", mi)
		if mi > best_mi
			best_mi = mi
			best_a = a
		end
	end
	return best_a
end
