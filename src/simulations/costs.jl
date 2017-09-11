######################################################################
# costs.jl
#
# TODO: export CostModel so people can extend it?
######################################################################
export ConstantCost, MoveCost, MoveCost2, MoveAndRotateCost

abstract type CostModel end

type ConstantCost <: CostModel
	value::Float64
end

type MoveCost <: CostModel
	speed::Float64

	MoveCost(speed::Real) = new(float(speed))
	MoveCost() = new(1.0)
end

type MoveCost2 <: CostModel
	speed::Float64

	MoveCost2(speed::Real) = new(float(speed))
	MoveCost2() = new(1.0)
end

type MoveAndRotateCost <: CostModel
	speed::Float64
	time_per_rotation::Float64
end

function get_action_cost(a::Action, cm::CostModel)
	error("get_action_cost not implemented for this cost model type.")
end
function get_action_cost(cm::CostModel)
	error("get_action_cost not implemented for this cost model type.")
end


function get_action_cost(a::Action, cc::ConstantCost)
	return cc.value
end
get_action_cost(cc::ConstantCost) = cc.value

function get_action_cost(a::Action, mc::MoveCost)
	dx = a[1]
	dy = a[2]
	dist = sqrt(dx*dx + dy*dy)
	return (dist / mc.speed)
end

# TODO: should I incorporate speed somehow?
function get_action_cost(a::Action, mc::MoveCost2)
	dx = a[1]
	dy = a[2]
	return dx*dx + dy*dy
end

function get_action_cost(a::Action, marc::MoveAndRotateCost)
	dx = a[1]
	dy = a[2]
	dist = sqrt(dx*dx + dy*dy)
	return (dist / marc.speed) + marc.time_per_rotation
end
get_action_cost(marc::MoveAndRotateCost) = marc.time_per_rotation
