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
function update!(su::SimUnit, o)
    update!(su.f, su.x, o)
end
function action(m::SearchDomain, su::SimUnit, o)
    return action(m, su.x, o, su.f, su.p)
end


function simulate(m::SearchDomain, uav::SimUnit)

    # reset the filter, vehicle, and policy
    # TODO: I think I assume the SimUnit comes in clean and ready to go
    #reset!(uav.f)
    #reset!(m, uav.x)
    #reset!(uav.p)

    # What was the cost to getting this first observation?
    cost_sum = get_cost(uav, m)

    # before doing anything else, we observe
    #  and update filter once
    o = observe(m, uav.x)
    update!(uav, o)

    # This was our first step; steps count number of observations
    step_count = 1

    while !is_complete(uav.f, uav.tc, step_count)
        # act
        a = action(m, uav, o)
        act!(m, uav.x, a)

        # get cost and update step count
        cost_sum += get_cost(uav, m, a)
        step_count += 1

        # observe and update
        o = observe(m, uav.x)
        update!(uav, o)
    end

    return cost_sum
end
