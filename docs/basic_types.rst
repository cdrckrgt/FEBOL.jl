==================
Basic Types
==================

Search Domain
===============
::
    
    # puts target at (tx, ty)
    m = SearchDomain(side_length, tx, ty)

    # puts target at random location within the square domain
    m = SearchDomain(side_length)

LocTuple
===========
::

    const LocTuple = NTuple{2, Float64}

Pose
===========
::

    const Pose = NTuple{3, Float64}

Action
===========
::

    const Action = NTuple{3, Float64}
