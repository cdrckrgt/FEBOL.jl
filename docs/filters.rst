=========================
Filters
=========================

A filter is something that maintains a belief over the search space and updates it given new observations and vehicle locations.

Note that each filter maintains a belief, which is a questionable design decision.
In reality, a belief is something separate, fed into a filter to be updated.
However, the belief representation (discrete, Gaussian, etc) depends heavily on the filtering being applied.
In short, it just seems easier to maintain a single filter type rather than worry about a separate belief.

Note that each filter has its own sensor, even though the vehicle also has a sensor.
The filtering updates use the filter's sensor, and the observations actually received come from the vehicle's sensor.
This distinction allows you to test the effect of unmodeled sensor noise.
In this case, the vehicle's sensor might have noise that is not accounted for in the filter's model, which can affect localization.

Discrete Filter
=====================
The discrete filter type, :code:`DF`, has the following fields
::

    b::Matrix{Float64}      # the actual discrete belief
    n::Int64                # number of cells per side
    cell_size::Float64      # width of each cell, in meters
    sensor<:Sensor          # sensor model used in filtering
    obs_list                # list of observations

The matrix :code:`b` is the probability distribution over possible target locations.
The weight in a cell is the probability that the target is in that cell.

The :code:`obs_list` field exists for greedy control based on mutual information.
Computing mutual information requires integrating over possible observations.
However, if you are using a different controller you can ignore this field.

The constructor for a discrete filter is
::

    DF(m::SearchDomain, n::Int, s::Sensor, obs_list=0:0)

where :code:`n` is the number of cells per side.


Particle Filter
=====================
The particle filter is based on ParticleFilters.jl.
Its constructor is
::

    PF(m::Model, n::Int, obs_list)

The :code:`Model` type contains information that is used in the particle filter update.
The type and constructors are
::

    struct Model{V <: Vehicle, S <: Sensor, M <: MotionModel}
        x::V
        sensor::S
        motion_model::M
    end
    Model(x::Vehicle) = Model(x, x.sensor)
    Model(x::Vehicle, s::Sensor) = Model(x, s, NoMotion())


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

The :code:`GaussianFilter` abstract type covers utilities that both :code:`EKF` and :code:`UKF` use.
The most important of these is the :code:`Initializer` abstract type.
Each :code:`EKF` and :code:`UKF` instance contains an :code:`Initializer` subtype that determines how the filter estimate should be initialized.

The default initializer is a :code:`NaiveInitializer` sets the estimate to be the center of the search domain and uses a large initial covariance.

Another initializer is the :code:`LSInitializer`, or least squares initializer. After taking :code:`min_obs_num` observations, this initializer sets the mean to the point in the search domain yielding the smallest sum of least square differences between observed and expected observations. The code below shows how to initialize an instance of :code:`LSInitializer` and modify some of its important fields:
::

    lsi = LSInitializer(m::SearchDomain)
    lsi.Sigma = 1e3*eye(2)
    lsi.min_obs_num = 5


    

Custom Filters
=====================
The code below is a template for creating your own filter type.
You must extend the :code:`AbstractFilter` type and implement the following functions.
::

    type CustomFilter <: AbstractFilter
    end

    function update!(f::CustomFilter, p::Pose, o::Float64)
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
