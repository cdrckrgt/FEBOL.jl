######################################################################
# simunit.jl
######################################################################

export SimUnit

"""
type `SimUnit` has fields
* x::Vehicle
* f::AbstractFilter
* p::Policy
* cm::CostModel

Constructor: `SimUnit(x, f, p, cm)`
"""
type SimUnit
	x::Vehicle
	f::AbstractFilter
	p::Policy
	cm::CostModel

	function SimUnit(x::Vehicle, f::AbstractFilter, p::Policy, cm::CostModel=ConstantCost(1.0))
		return new(x,f,p,cm)
	end
end

function simulate(m::SearchDomain, uav::SimUnit, tc::TerminationCondition; video::Bool=false, pause_time=0.3)

	# reset the filter, vehicle, and policy
	# TODO: I think I assume the SimUnit comes in clean and ready to go
	#reset!(uav.f)
	#reset!(m, uav.x)
	#reset!(uav.p)

	# before doing anything else, we observe
	#  and update filter once
	o = observe(m, uav.x)
	update!(uav, o)

	# This was our first step; steps count number of observations
	step_count = 1

	# plot if need be
	if video
		plot(m, uav.f, uav.x)
		title("i = $(step_count)")
	end

	# What was the cost to getting this first observation?
	temp_cost = get_action_cost(uav.cm)

	while !is_complete(uav.f, tc, step_count)
		# act
		a = action(m, uav, o)
		act!(m, uav.x, a)

		# get cost and update step count
		temp_cost += get_action_cost(a, uav.cm)
		step_count += 1

		# observe and update
		o = observe(m, uav.x)
		update!(uav, o)

		# plot if need be
		if video
			pause(pause_time)
			cla()
			plot(m, uav.f, uav.x)
			max_b = maximum(uav.f.b)
			title("i = $(step_count), max = $(round(max_b,3))")
		end
	end

	return temp_cost
end
