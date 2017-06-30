================
Vehicle
================

Each instance of :code:`Vehicle` has the following fields:
::

    x::Float64
    y::Float64			 
    heading::Float64	# east of north (degrees)
    max_step::Float64	# max distance vehicle can go per unit time (meters)
    sensor::Sensor

There are several constructors. Below is the default:
::

    v = Vehicle(x::Real, y::Real, h::Real, ms::Real, s::Sensor)

If you just give it a starting location, :code:`heading` is set to 0, :code:`max_step` is set to 2.0, and the :code:`sensor` is defaulted to :code:`BearingOnly(10.0)` (a bearing-only sensor with noise std deviation of 10 deg).
::

    v = Vehicle(x::Real, y::Real)

Alternatively, you can pass the sensor in as well, with the omitted variables as above:
::

    v = Vehicle(x::Real, y::Real, s::Sensor)
