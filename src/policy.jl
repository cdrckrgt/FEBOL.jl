######################################################################
# policy.jl
# Does basic policy stuff.
# Policy's don't need to remember vehicle's max_step,
#  as the action function ensures the step is normalized to max_step.
######################################################################

abstract Policy

"""
`action(m, x, o, f, p)`

Returns an action given SearchDomain `m`, vehicle `x`, observation `o`, filter `f`, and policy `p`.
"""
function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::Policy)
	return error(typeof(p), " does not yet implement action.")
end

# TODO: should ensure the heading is not too much either
function normalize(a::Action, x::Vehicle)
	ax = a[1]; ay = a[2]
	den = sqrt(ax*ax + ay*ay)
	return x.max_step * ax / den, x.max_step * ay / den, a[3]
end


# Random policy
type RandomPolicy <: Policy end

function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::RandomPolicy)
	ax = rand() - 0.5
	ay = rand() - 0.5
	return normalize((ax,ay), x)
end

# Sit policy
type SitPolicy <: Policy end

action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::SitPolicy) = (0.0,0.0)


# Greedy info-theoretic policy
# only consider a set of possible actions
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


# Move orthogonally to last measurement
type CirclePolicy <: Policy 
	last::Action

	CirclePolicy() = new( (0.0, 0.0, 0.0) )
end

# Remembers last action to ensure we follow same direction around circle
# Otherwise, it will "chatter" back and forth
# TODO: take into account distance from edge
function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::CirclePolicy)
	ax = -1.0 / sind(o)
	ay = 1.0 / cosd(o)

	if ax*p.last[1] + ay*p.last[2] < 0.
		ax = -ax
		ay = -ay
	end

	p.last = (ax, ay, 0.0)

	return normalize((ax,ay,0.0), x)
end


# Move orthogonally to last measurement
type SpiralPolicy <: Policy 
	last::Action

	SpiralPolicy() = new( (0.0, 0.0) )
end

# Remembers last action to ensure we follow same direction around circle
# Otherwise, it will "chatter" back and forth
# TODO: take into account distance from edge
function action(m::SearchDomain, x::Vehicle, o::Float64, f::DF, p::SpiralPolicy)
	ax = -1.0 / sind(o)
	ay = 1.0 / cosd(o)

	if ax*p.last[1] + ay*p.last[2] < 0.
		ax = -ax
		ay = -ay
	end

	p.last = (ax, ay)

	return normalize((ax,ay), x)
end



