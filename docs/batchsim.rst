====================
Batch Simulations
====================
Sometimes, we want to run a batch of simulations to evaluate the average performance of a sensor and policy.
Sometimes we want to test multiple different sensors and policies at the same time.

FEBOL's framework performing these batch simulations is the :code:`batchsim` function, which has the following signature:

::

    batchsim(m::SearchDomain, vsu::Vector{SimUnit}, num_sims::Int, tc::TerminationCondition)


Simulation Unit
==================
::

    type SimUnit
        x::Vehicle
        f::AbstractFilter
        p::Policy
        cm::CostModel

        function SimUnit(x::Vehicle, f::AbstractFilter, p::Policy, cm::CostModel=ConstantCost(1.0))
            return new(x,f,p,cm)
        end
    end


Cost Model
==============
The abstract :code:`CostModel` type handles how costs are applied throughout the simulations.

To define your own 
::

    type CustomCost <: CostModel
        # whatever fields you need for get_action_cost
    end
    function get_action_cost(a::Action, cm::CostModel)
        # return a Float64 describing cost
    end

For example, the :code:`ConstantCost` model has the following.
::

    type ConstantCost <: CostModel
        value::Float64
    end

    function get_action_cost(a::Action, cc::ConstantCost)
        return cc.value
    end

The :code:`MoveAndRotateCost` is a more complex example.
::

    type MoveAndRotateCost <: CostModel
        speed::Float64
        time_per_rotation::Float64
    end

    function get_action_cost(a::Action, marc::MoveAndRotateCost)
        dx = a[1]
        dy = a[2]
        dist = sqrt(dx*dx + dy*dy)
        return (dist / marc.speed) + marc.time_per_rotation
    end


Termination Condition
=======================

