######################################################################
# costs.jl
######################################################################
export ConstantCost, MoveAndRotateCost

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
