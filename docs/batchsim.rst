====================
Batch Simulations
====================
Sometimes, we want to run a batch of simulations to evaluate the average performance of a sensor and policy.
Sometimes we want to test multiple different sensors and policies at the same time.

FEBOL's framework performing these batch simulations is the :code:`batchsim` function, which has the following signature:

::

    batchsim(m::SearchDomain, vsu::Vector{SimUnit}, num_sims::Int, tc::TerminationCondition)

What this returns is

Simulation Unit
==================
::

    type SimUnit
        x::Vehicle
        f::AbstractFilter
        p::Policy
        cm::CostModel
        tc::TerminationCondition

        function SimUnit(x::Vehicle, f::AbstractFilter, p::Policy, tc::TerminationCondition = StepThreshold(10), cm::CostModel=ConstantCost(1.0))
            return new(x,f,p,cm)
        end
    end

Below are the constructors for the :code:`SimUnit` type. At a minimum, it needs a vehicle, filter, and policy. If no cost model is provided, it defaults to :code:`ConstantCost(1.0)`. If no termination condition is provided, it defaults to :code:`StepThreshold(10)`.

::
    
    SimUnit(x, f, p)            # default termination and cost
    SimUnit(x, f, p, tc)        # default cost
    SimUnit(x, f, p, tc, cm)    # fully defined


Cost Model
==============
The abstract :code:`CostModel` type handles how costs are applied throughout the simulations.
Two cost models are provided:

To define your own cost model, you must extend the abstract :code:`CostModel` type and implement the :code:`get_action_cost` function.
::

    type CustomCost <: CostModel
        # whatever fields you need for get_action_cost
    end
    function get_action_cost(a::Action, cm::CostModel)
        # return a Float64 describing cost
    end

For an example, let's examine the :code:`ConstantCost` model, which applies the same cost at each step.
This cost might represent the time each step takes.
Therefore, a simulated trajectory's cost would simulate how much time it took.
The :code:`ConstantCost` model is defined as follows,
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
The abstract :code:`TerminationCondition` type determines when an individual simulation should be terminated.


To define your own termination condition, you must extend the abstract :code:`TerminationCondition` type and implement the :code:`is_complete` function.
::

    type CustomTC <: TerminationCondition
        # whatever fields you need
    end

    function is_complete(f::AbstractFilter, ctc::CustomTC, step_count::Int)
        # return true if termination condition reached, false if not
    end

The :code:`step_count` argument is passed in by the thing.
(Clarify if it starts at one or zero.)
You can define the :code:`is_complete` function for a specific kind of filter if you only plan on using one filter.

The :code:`StepThreshold` is provided.
It terminates after a specified number of steps has been simulated.
::

    type StepThreshold <: TerminationCondition
        value::Int
    end
    function is_complete(f::DF, st::StepThreshold, step_count::Int)
        ret_val = false
        if step_count >= st.value
            ret_val = true
        end
        return ret_val
    end

The :code:`MaxNormThreshold` termination condition is also provided.
The implementation is below
::

    type MaxNormThreshold <: TerminationCondition
        value::Float64
    end

    function is_complete(f::DF, mnt::MaxNormThreshold, ::Int)
        ret_val = false
        if maximum(f.b) > mnt.value
            ret_val = true
        end
        return ret_val
    end
