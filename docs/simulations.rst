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

    steps!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int)

Alternatively, you can just call :code:`steps!()`.
This will run a simulation for 10 steps. 
It assumes you have a model :code:`m`, a Vehicle :code:`x`, a filter :code:`f`, and a policy :code:`p` in the main module (with those names).
If you've named your objects that way, you don't have to remember the arguments required for :code:`steps!`.
Alternatively, you can specify a number of steps: :code:`steps!(30)`.


Creating GIFs
=================
::

    gif(m, x, f, p, num_steps)


::

    gif(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int=10, filename="out.gif"; seconds_per_step=0.5, show_mean=false, show_cov=false, show_path=false)
    
