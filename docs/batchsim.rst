====================
Batch Simulations
====================
Sometimes, we want to run a batch of simulations to evaluate the average performance of a sensor and policy.
Sometimes we want to test multiple different sensors and policies at the same time.

FEBOL's framework performing these batch simulations is the :code:`batchsim` function, which has the following signature:

::

    function batchsim(m::SearchDomain, uav_array::Vector{SimUnit}, num_sims::Int, tc::TerminationCondition)


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
