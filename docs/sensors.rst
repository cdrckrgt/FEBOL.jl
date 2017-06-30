====================
Sensors
====================

The abstract :code:`Sensor` type describes the sensing model of the vehicle.
Originally, the only sensor type was bearing only, but this has been expanded to consider other sensing modalities.

BearingOnly
=================
::

    BearingOnly(noise_sigma)


DirOmni
==========
The :code:`DirOmni` sensor combines a directional antenna with an omni-directional antenna.

FOV
==========
The :code:`FOV` sensor is a "field-of-view" sensor. The observed value 1 suggests the source is in the vehicle's field of view, and 0 suggests the source is not.
::

    region_probs = [(60.0,0.9), (120.0, 0.5), (180.0,0.1)]
    sensor = FOV(region_probs)
    v = Vehicle(50,50, sensor)
