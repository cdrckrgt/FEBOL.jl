######################################################################
# policy.jl
# Does basic policy stuff.
######################################################################

abstract Policy

function action(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy)
	return error("$(typeof(p)) does not yet implement action.")
end


# Random policy
type RandomPolicy <: Policy
	max_step::Float64

	RandomPolicy(ms::Float64) = new(ms)
	Randomlicy(x::Vehicle) = new(x.max_step)
	RandomPolicy() = new(2.0)
end

function action(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::RandomPolicy)
	ax = rand() - 0.5
	ay = rand() - 0.5

	den = sqrt(ax*ax + ay*ay)
	ax = x.max_step * ax / den
	ay = x.max_step * ay / den
	return ax, ay
end


# Greedy info-theoretic policy
# only consider a set of possible actions
type GreedyPolicy <: Policy
	n::Int
	actions::Vector{Action}
	max_step::Float64

	function GreedyPolicy(x::Vehicle, n::Int)
		angles = linspace(0.0, 360 - 360/n, n)

		# create list of actions
		actions = Array(Action, n)
		for i = 1:n
			ax = x.max_step * sind(angles[i])
			ay = x.max_step * cosd(angles[i])
			actions[i] = (ax, ay)
		end

		return new(n, actions, x.max_step)
	end
end

# loop over all actions.
# The one with smallest expected entropy is best
function action(m::SearchDomain, x::Vehicle, f::DF, p::GreedyPolicy)
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
