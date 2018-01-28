#####################################################################
# simulate.jl
######################################################################

# sample an observation
#  1. receives an observation
#  2. updates belief (contained in filter)
#  3. updates vehicle position according to some policy
# For right now, assume a random policy
function step!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy)
	# observe, update belief, select action, and act
	o = observe(m,x)
	update!(f, x, o)
	a = action(m, x, o, f, p)
	act!(m,x,a)
end

function simulate(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, tc::TerminationCondition=StepThreshold(10))
	# Then go through steps
	step_count = 0
	while !is_complete(f, tc, step_count)
		step!(m,x,f,p)
		step_count += 1
	end
end
