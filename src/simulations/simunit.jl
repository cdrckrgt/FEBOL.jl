"""
type `SimUnit` has fields
* x::Vehicle
* f::AbstractFilter
* p::Policy
* cm::CostModel

Constructor: `SimUnit(x, f, p, cm)`
"""
struct SimUnit
    x::Vehicle
    f::AbstractFilter
    p::Policy
    cm::CostModel
    tc::TerminationCondition
end
function SimUnit(x::Vehicle,
                 f::AbstractFilter,
                 p::Policy,
                 tc::TerminationCondition = StepThreshold(10),
                 cm::CostModel = ConstantCost(1.0)
                )
    return SimUnit(x, f, p, cm, tc)
end

get_cost(su::SimUnit, m) = get_cost(su.cm, m, su.x, su.f)
get_cost(su::SimUnit, m, a::Action) = get_cost(su.cm, m, su.x, su.f, a)

function reset!(m::SearchDomain, su::SimUnit)
    reset!(su.f)
    reset!(m, su.x)
    reset!(su.p)
end

# The following are just helper functions that simplify signatures
update!(su::SimUnit, o) = update!(su.f, su.x, o)
action(m::SearchDomain, su::SimUnit, o) = action(m, su.x, o, su.f, su.p)

function is_complete(vsu::Vector{SimUnit}, step_count::Int)
    ret_val = false
    for uav in vsu
        if is_complete(uav.f, uav.tc, step_count)
            ret_val = true
        end
    end
    return ret_val
end
