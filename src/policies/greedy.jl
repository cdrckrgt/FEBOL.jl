######################################################################
# greedy.jl
#
# Defines a greedy, info-theoretic policy
# Currently only considers a small set of the possible actions
######################################################################
type GreedyPolicy <: Policy
	n::Int
	actions::Vector{Action}

	function GreedyPolicy(x::Vehicle, n::Int)
		angles = linspace(0.0, 360 - 360/n, n)

		# create list of actions
		actions = Array(Action, n)
		for i = 1:n
			ax = x.max_step * sind(angles[i])
			ay = x.max_step * cosd(angles[i])
			actions[i] = (ax, ay)
		end

		return new(n, actions)
	end
end


# loop over all actions.
# The one with smallest expected entropy is best
function action(m::SearchDomain, x::Vehicle, o::Float64, f::DF, p::GreedyPolicy)
	best_mi = -Inf
	best_a = (0.0, 0.0)
	for a in p.actions
		# find out where a will take you
		xv, yv = new_location(m, x, a)
		# compute best maximum information
		mi = mutual_information(m,f,xv,yv)
		if mi > best_mi
			best_mi = mi
			best_a = a
		end
	end
	return best_a
end
