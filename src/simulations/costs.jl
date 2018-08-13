######################################################################
# costs.jl
#
######################################################################

abstract type CostModel end

function get_cost(cm::CostModel, m::SearchDomain, x::Vehicle, f::AbstractFilter, a)
    error("get_cost not implemented for this cost model type.")
end
function get_cost(cm::CostModel, m::SearchDomain, x::Vehicle, f::AbstractFilter)
    error("get_cost not implemented for this cost model type.")
end


struct ConstantCost <: CostModel
    value::Float64
end
function get_cost(cc::ConstantCost,
                  m::SearchDomain,
                  x::Vehicle,
                  f::AbstractFilter,
                  a
                 )
    return cc.value
end
get_cost(cc::ConstantCost, m::SearchDomain, x::Vehicle, f::AbstractFilter) = cc.value


struct MoveCost <: CostModel
    speed::Float64

    MoveCost(speed::Real) = new(float(speed))
    MoveCost() = new(1.0)
end
function get_cost(mc::MoveCost, x::Vehicle, f::AbstractFilter, a)
    dx = a[1]
    dy = a[2]
    dist = sqrt(dx*dx + dy*dy)
    return (dist / mc.speed)
end


struct MoveCost2 <: CostModel
    speed::Float64

    MoveCost2(speed::Real) = new(float(speed))
    MoveCost2() = new(1.0)
end

# TODO: should I incorporate speed somehow?
function get_cost(mc::MoveCost2, x::Vehicle, f::AbstractFilter, a)
    dx = a[1]
    dy = a[2]
    return dx*dx + dy*dy
end


# Move and rotate
struct MoveAndRotateCost <: CostModel
    speed::Float64
    time_per_rotation::Float64
end
function get_cost(marc::MoveAndRotateCost, x::Vehicle, f::AbstractFilter, a)
    dx = a[1]
    dy = a[2]
    dist = sqrt(dx*dx + dy*dy)
    return (dist / marc.speed) + marc.time_per_rotation
end
get_cost(marc::MoveAndRotateCost, f::AbstractFilter) = marc.time_per_rotation

export EntropyCost
struct EntropyCost <: CostModel
    L::Float64
    n_cells::Int
end

function get_cost(ec::EntropyCost, m::SearchDomain, x::Vehicle, f::PF, a)
    get_cost(ec, m, x, f)
end
function get_cost(ec::EntropyCost, m::SearchDomain, x::Vehicle, f::PF)
    cheap_entropy(f, ec.L, ec.n_cells)
    #c = covariance(f)
    #earr = eigvals(c)
    #return maximum(eigvals(c))
end
