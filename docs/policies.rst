==================
Policies
==================

RandomPolicy
===============
A :code:`RandomPolicy` simply moves the vehicle in a random direction.
::

    RandomPolicy()


GreedyPolicy
===================
A :code:`GreedyPolicy` moves the agent in the direction that minimizes the expected entropy after moving.
::

    GreedyPolicy(x::Vehicle, n::Int)

The integer :code:`n` denotes how many actions should be considered.
If :code:`n=6`, then the agent considers the expected entropy given 6 different directions, spaced an even 60 degrees apart.

CirclePolicy
===================
A :code:`CirclePolicy` moves the agent perpendicularly to the last recorded bearing measurement, which ends up drawing a circle around the source.
The constructor is as follows:
::

    CirclePolicy()

The :code:`CirclePolicy` implicitly assumes that the sensor is of :code:`BearingOnly` type.

Custom Policy
===================
You can create your own policies by extending the abstract :code:`Policy` class and implementing the :code:`action` function. Below is an example. Remember that to extend :code:`FEBOL`'s :code:`action` function, you must import it instead of just relying on :code:`using`:
::

    using FEBOL
    import FEBOL.action

    type CustomPolicy <: Policy
    end

    function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::CustomPolicy)
        # your policy code
        # must return action (2-tuple of Float64s)
    end

Feel free to take advantage of the :code:`normalize` function to ensure your action's norm is equal to the maximum distance the vehicle can take per time step:
::

    normalize(a::Action, x::Vehicle)
