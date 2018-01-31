=================
Visualizatons
=================

Recall that visualizations require the FEBOLPlots.jl package. To install this package, you must call the following in Julia:
::

    Pkg.clone("https://github.com/dressel/FEBOLPlots.jl.git")

Once the package has been installed, you must include the statement :code:`using FEBOLPLots` whenever using one of its functions.
The most useful functions will be :code:`visualize` and :code:`gif`.

    

Visualize Function
===================
The :code:`visualize` function allows you to plot out several steps.
A simple version can be called with

::
    
    visualize(m, x, f, p, n_steps=10; pause_time=0.3)

where :code:`m` is a SearchDomain, :code:`x` is a Vehicle, :code:`f` is a filter, and :code:`p` is a policy.

A different version allows you to pass in :code:`SimUnit`:

::
    
    visualize(m::SearchDomain, su::SimUnit; pause_time=0.3)

Creating GIFs
=================
::

    gif(m, x, f, p, num_steps)


::

    gif(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int=10, filename="out.gif"; seconds_per_step=0.5, show_mean=false, show_cov=false, show_path=false)
    
