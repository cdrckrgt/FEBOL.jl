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
