######################################################################
# costs.jl
#
######################################################################
export CostModel, ConstantCost, MoveCost, MoveCost2, MoveAndRotateCost

abstract type CostModel end

struct ConstantCost <: CostModel
    value::Float64
end

struct MoveCost <: CostModel
    speed::Float64

    MoveCost(speed::Real) = new(float(speed))
    MoveCost() = new(1.0)
end

struct MoveCost2 <: CostModel
    speed::Float64

    MoveCost2(speed::Real) = new(float(speed))
    MoveCost2() = new(1.0)
end

struct MoveAndRotateCost <: CostModel
    speed::Float64
    time_per_rotation::Float64
end

function get_cost(cm::CostModel, f::AbstractFilter, a::Action)
    error("get_cost not implemented for this cost model type.")
end
function get_cost(cm::CostModel, f::AbstractFilter)
    error("get_cost not implemented for this cost model type.")
end


function get_cost(cc::ConstantCost, f::AbstractFilter, a::Action)
    return cc.value
end
get_cost(cc::ConstantCost, f::AbstractFilter) = cc.value

function get_cost(mc::MoveCost, f::AbstractFilter, a::Action)
    dx = a[1]
    dy = a[2]
    dist = sqrt(dx*dx + dy*dy)
    return (dist / mc.speed)
end

# TODO: should I incorporate speed somehow?
function get_cost(mc::MoveCost2, f::AbstractFilter, a::Action)
    dx = a[1]
    dy = a[2]
    return dx*dx + dy*dy
end

function get_cost(marc::MoveAndRotateCost, f::AbstractFilter, a::Action)
    dx = a[1]
    dy = a[2]
    dist = sqrt(dx*dx + dy*dy)
    return (dist / marc.speed) + marc.time_per_rotation
end
get_cost(marc::MoveAndRotateCost, f::AbstractFilter) = marc.time_per_rotation
