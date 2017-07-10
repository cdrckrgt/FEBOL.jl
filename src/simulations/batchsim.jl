######################################################################
# batchsim.jl
#
# running many simulations
######################################################################
export SimUnit

######################################################################
# Simulation unit
######################################################################
type SimUnit
	x::Vehicle
	f::AbstractFilter
	p::Policy
	cm::CostModel

	function SimUnit(x::Vehicle, f::AbstractFilter, p::Policy, cm::CostModel=ConstantCost(1.0))
		return new(x,f,p,cm)
	end
end

# The following are just helper functions that simplify signatures
function update!(su::SimUnit, o::Float64)
	update!(su.f, su.x, o)
end
function action(m::SearchDomain, su::SimUnit, o::Float64)
	return action(m, su.x, o, su.f, su.p)
end


######################################################################
# simulation code
######################################################################

function batchsim(m::SearchDomain, uav_array::Vector{SimUnit}, num_sims::Int, tc::TerminationCondition)
	costs = zeros(num_sims, length(uav_array))
	for sim_ind = 1:num_sims
		theta!(m)
		print("Simulation ", sim_ind, ": ")
		for (uav_ind, uav) in enumerate(uav_array)
			print(uav_ind, ",")

			# reset the filter, vehicle, and policy
			reset!(uav.f)
			reset!(m, uav.x)
			reset!(uav.p)

			# starts at -1 because after first round, should be zero
			temp_cost = -1.0
			step_count = 0
			while !is_complete(uav.f, tc, step_count)
				# observe, update filter, act
				o = observe(m, uav.x)
				update!(uav, o)
				a = action(m, uav, o)
				act!(m, uav.x, a)

				# get cost and update step count
				temp_cost += get_action_cost(a, uav.cm)
				step_count += 1
			end

			costs[sim_ind, uav_ind] = temp_cost
		end
		println("complete.")
	end
	return costs
end
