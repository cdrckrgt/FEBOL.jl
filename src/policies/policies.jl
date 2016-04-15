######################################################################
# policies.jl
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


######################################################################
# greedy policy
######################################################################
include("greedy.jl")


######################################################################
# Random policy
######################################################################
type RandomPolicy <: Policy end

function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::RandomPolicy)
	ax = rand() - 0.5
	ay = rand() - 0.5
	ah = 10.0
	if ax < 0.0
		ah = -10.0
	end
	return normalize((ax,ay,ah), x)
end


######################################################################
# Sit policy: Just sit and don't move
######################################################################
type SitPolicy <: Policy end

action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::SitPolicy) = (0.0,0.0,0.0)


######################################################################
# Spin Policy: Just sit there and spin
######################################################################
type SpinPolicy <: Policy end

function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::SpinPolicy)
	return (0.0, 0.0, 10.0)
end


######################################################################
# CirclePolicy: Move orthogonally to last measurement

# This ends up tracing a circle around the jammer
######################################################################
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


######################################################################
# SpiralPolicy: Like CirclePolicy, but also move towards jammer
# 
# TODO: actually implement this
######################################################################
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
