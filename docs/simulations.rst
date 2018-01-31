===============
Simulations
===============

Quick Simulations
=====================
If you've just implemented a sensor, filter, or policy, you might want to run it through a quick simulation to make sure everything works.
You can simply call

::

    simulate(m, x, f, p, n_steps=10)

where :code:`m` is a SearchDomain, :code:`x` is a Vehicle, :code:`f` is a filter, and :code:`p` is a policy.
If everything works, no error will be thrown.

Simulations with SimUnit
===========================

::
    
    simulate(m::SearchDomain, su::SimUnit)


Batch Simulations
=======================
Sometimes, we want to run a batch of simulations to evaluate the average performance of a sensor and policy.
We might even want to test different sensors and policies at the same time.

You can pass in a vector SimUnits to :code:`simulate` and provide the number of simulations you want to execute per SimUnit.
For each simulation, the target is initialized to a random location.
Then each SimUnit is run on this target location.

::

    simulate(m::SearchDomain, vsu::Vector{SimUnit}, n_sims::Int)

This returns a matrix with one row for each simulation and one column for each sim unit. In each simulation (a row), each simulation unit is tested with the same target location. The values in this matrix correspond to the total cost/reward accumulated druing the simulations.


Parallel Simulations
=========================
To devote :code:`n` cores to running simulations, you must start Julia with the following command
::
    
    julia -p n

To run simulations in parallel, use the :code:`parsim` function, which takes the same arguments as the :code:`simulate` function for batch simulations:
::
    
    parsim(m::SearchDomain, vsu::Vector{SimUnit}, n_sims::Int)


Under the Hood
=================


