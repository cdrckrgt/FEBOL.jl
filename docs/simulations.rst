===============
Simulations
===============

Simulating a Single Step
==========================
Suppose :code:`m` is a search domain, :code:`x` is a vehicle, :code:`f` is a filter, and :code:`p` is a policy.
You can simulate a step with the following code:
::

    o = observe(m, x)
    update!(f, x, o)
    a = action(m, x, o, f, p)
    act!(m,x,a)


The :code:`step!` function performs this code and also plots if the optional :code:`video` argument is set to true.
::

    step!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy; video::Bool=true)


Simulating Multiple Steps
===========================
::

    simulate(m, x, f, p, 10)

::
    
    simulate(m, su)


Batch Simulations
=======================
Sometimes, we want to run a batch of simulations to evaluate the average performance of a sensor and policy.
Sometimes we want to test multiple different sensors and policies at the same time.

FEBOL's framework performing these batch simulations is the :code:`batchsim` function, which has the following signature:

::

    batchsim(m::SearchDomain, vsu::Vector{SimUnit}, num_sims::Int)

This returns a matrix with one row for each simulation and one column for each sim unit. In each simulation (a row), each simulation unit is tested with the same target location. The values in this matrix correspond to the total cost/reward accumulated druing the simulations.

