===============
Simulations
===============
::

    steps!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int)

Alternatively, you can just call :code:`steps!()`.
This will run a simulation for 10 steps. 
It assumes you have a model :code:`m`, a Vehicle :code:`x`, a filter :code:`f`, and a policy :code:`p` in the main module (with those names).
If you've named your objects that way, you don't have to remember the arguments required for :code:`steps!`.
Alternatively, you can specify a number of steps: :code:`steps!(30)`.
