=========================
Filters
=========================

A filter is something that maintains a belief over the search space and updates it given new observations and vehicle locations.

Note that each filter maintains a belief, which is a questionable design decision.
In reality, a belief is something separate, fed into a filter to be updated.
However, the belief representation (discrete, Gaussian, etc) depends heavily on the filtering being applied.
In short, it just seems easier to maintain a single filter type rather than worry about a separate belief.

Discrete Filter
=====================
The constructor for a discrete filter is
::

    DF(m::SearchDomain, n::Int)

where :code:`n` is the number of cells per side.

If you create a new :code:`Sensor` subtype called :code:`NewSensor`, you need to implement the following methods for the :code:`DF` type to work:
::

    obs2bin(o::Float64, f::Filter, s::NewSensor)
    O(x::Vehicle, s::NewSensor, theta::LocTuple, ob::Int, f::Filter)

The function :code:`obs2bin` converts the continuous domain observation :code:`o` into a discrete value, using the :code:`num_bins` property of the discrete filter.
The binned observation :code:`ob` is fed into :code:`O`, which provides the probability of receiving :code:`ob`.
I've implicitly assumed that the observations are discrete in the discrete filter, but I think only the search domain needs to be discretized, technically.

Extended Kalman Fiter
===========================
::

    EKF(m::SearchDomain)


Unscented Kalman Fiter
=========================
::

    UKF(m::SearchDomain)


Gaussian Fiter
====================
The :code:`GaussianFilter` abstract type is a child of :code:`AbstractFilter` and a parent of :code:`EKF` and :code:`UKF`. I've thought about calling this :code:`KalmanFilter` instead, but that could be ambiguous---someone could think this refers to a specific KF, rather than an abstract type. 

The `GaussianFilter` abstract type covers utilities that both `EKF` and `UKF` use.
The most important of these is the :code:`Initializer` abstract type.
Each :code:`EKF` and :code:`UKF` instance contains an :code:`Initializer` subtype that determines how the filter estimate should be initialized.

The default initializer is a :code:`NaiveInitializer` sets the estimate to be the center of the search domain and uses a large initial covariance.

Another initializer is the :code:`LSInitializer`, or least squares initializer. After taking :code:`min_obs_num` observations, this initializer sets the mean to the point in the search domain yielding the smallest sum of least square differences between observed and expected observations. The code below shows how to initialize an instance of :code:`LSInitializer` and modify some of its important fields:
::

    lsi = LSInitializer(m::SearchDomain)
    lsi.Sigma = 1e3*eye(2)
    lsi.min_obs_num = 5


Particle Filter
=====================
To create a particle filter, you must provide the search domain :code:`m` and desired number of particles :code:`n`:
::

    PF(m::SearchDomain, n::Int)

If you create a new :code:`Sensor` subtype called :code:`NewSensor`, you must implement the following function:
::

    O(x::Vehicle, s::NewSensor, theta::LocTuple, o::Float64)

Currently, it is only required that this return a probability density, rather than a true probability.

Custom Filters
=====================
The code below is a template for creating your own filter type.
You must extend the :code:`AbstractFilter` type and implement the following functions.
::

    type CustomFilter <: AbstractFilter
    end

    function update!(f::CustomFilter, x::Vehicle, o::Float64)
        # update the belief in the filter.
    end

    function centroid(f::CustomFilter)
        # return the centroid of the filter's belief
    end

    function entropy(f::CustomFilter)
        # return the entropy of the filter's belief
    end

    function reset!(f::CustomFilter)
        # reset the filter to a uniform prior
    end
