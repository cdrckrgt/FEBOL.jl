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
		return GreedyPolicy(x, x.sensor, n)
	end
	function GreedyPolicy(x::Vehicle, ::BearingOnly, n::Int)

		angles = linspace(0.0, 360 - 360/n, n)

		# create list of actions
		actions = Array(Action, n+1)
		for i = 1:n
			ax = x.max_step * sind(angles[i])
			ay = x.max_step * cosd(angles[i])
			actions[i] = (ax, ay, 0.0)
		end
		actions[n+1] = (0.0, 0.0, 0.0)

		return new(n, actions)
	end

	function GreedyPolicy(x::Vehicle, ::DirOmni, n::Int)
		actions = make_action_list(x, n)
		return new(n, actions)
	end

	function GreedyPolicy(x::Vehicle, ::FOV, n::Int)
		actions = make_action_list(x, n)
		return new(n, actions)
	end
end

function make_action_list(x::Vehicle, n::Int)
	angles = linspace(0.0, 360 - 360/n, n)
	d_angle = 10.0

	# create list of actions
	actions = Array(Action, 3n+3)
	for i = 1:n
		ax = x.max_step * sind(angles[i])
		ay = x.max_step * cosd(angles[i])

		actions[i] = (ax, ay, -d_angle)
		actions[i+n] = (ax, ay, 0.0)
		actions[i+2n] = (ax, ay, d_angle)
	end
	actions[3n+1] = (0.0, 0.0, -d_angle)
	actions[3n+2] = (0.0, 0.0, 0.0)
	actions[3n+3] = (0.0, 0.0, d_angle)

	return actions
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
