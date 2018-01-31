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
To specify costs and termination conditions, use the SimUnit type.

::
    
    simulate(m::SearchDomain, su::SimUnit)

This returns the total cost of the simulated run (a float).


Batch Simulations
=======================

To evaluate a SimUnit over the course of various simulations, you can provide a number of simulations, :code:`n_sims`, to :code:`simulate`:

::

    simulate(m::SearchDomain, su::SimUnit, n_sims::Int)

At the beginning of each simulation, the target is started in a random location.
The return value is a vector of cost values.
This vector is of length :code:`n_sims` and has one cost per simulation.

If we want to compare different filters and policies, we can provide a vector of SimUnits to :code:`simulate`:

::

    simulate(m::SearchDomain, vsu::Vector{SimUnit}, n_sims::Int)

A total of :code:`n_sims` simulations is run per SimUnit.
Once a new (random) target location is selected, all SimUnits are run once.
The return value is a matrix with one row for each simulation and one column for each sim unit. In each simulation (a row), each simulation unit is tested with the same target location. The values in this matrix correspond to the total cost/reward accumulated druing the simulations.


Parallel Simulations
=========================
To devote :code:`n` cores to running simulations, you must start Julia with the following command
::
    
    julia -p n

To run simulations in parallel, use the :code:`parsim` function, which takes the same arguments as the :code:`simulate` function for batch simulations:
::
    
    parsim(m::SearchDomain, su::SimUnit, n_sims::Int)
    parsim(m::SearchDomain, vsu::Vector{SimUnit}, n_sims::Int)


Under the Hood
=================


