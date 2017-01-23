######################################################################
# batchsim.jl
#
# running many simulations
######################################################################
export SimUnit
export MaxNormThreshold, is_complete
export ConstantCost, MoveAndRotateCost

######################################################################
# termination condition stuff
######################################################################
abstract TerminationCondition

type EntropyThreshold <: TerminationCondition
	value::Float64
end
type MaxNormThreshold <: TerminationCondition
	value::Float64
end

# determines if a simulation has reached its 
function is_complete(f::AbstractFilter, ::TerminationCondition)
	error("Termination condition not implemented for this filter or termination condition")
end
function is_complete(f::DF, mnt::MaxNormThreshold)
	ret_val = false
	if maximum(f.b) > mnt.value
		ret_val = true
	end
	return ret_val
end


######################################################################
# costs
######################################################################
abstract CostModel

type ConstantCost <: CostModel
	value::Float64
end

type MoveAndRotateCost <: CostModel
	speed::Float64
	time_per_rotation::Float64
end

function get_action_cost(a::Action, cm::CostModel)
	error("get_action_cost not implemented for this cost model type.")
end

function get_action_cost(a::Action, cc::ConstantCost)
	return cc.value
end

function get_action_cost(a::Action, marc::MoveAndRotateCost)
	dx = a[1]
	dy = a[2]
	dist = sqrt(dx*dx + dy*dy)
	return (dist / marc.speed) + marc.time_per_rotation
end



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

			# reset the filter and vehicle
			reset!(uav.f)
			reset!(m, uav.x)

			# starts at -1 because after first round, should be zero
			temp_cost = -1.0
			while !is_complete(uav.f, tc)
				# observe, update filter, act
				o = observe(m, uav.x)
				update!(uav, o)
				a = action(m, uav, o)
				act!(m, uav.x, a)
				temp_cost += get_action_cost(a, uav.cm)
			end

			costs[sim_ind, uav_ind] = temp_cost
		end
		println("complete.")
	end
	return costs
end
