# FEBOL

This package allows the testing of various filters and belief representations for a single UAV performing bearing-only localization of a single, stationary jammer.
The main question I want to answer is: what effects do the various filters and approximations have on localization time and computational time?

This code is rapidly changing. I make no guarantees about its correctness or usability at any time.

## SearchDomain
To create a 10m by 10m search domain with a jammer located at 9.5, 9.5 you can do the following:
```
m = SearchDomain(10.0, 9.5, 9.5)
```
The jammer must be within 0 and the length of one side of the domain.


## Vehicle
Each instance of `Vehicle` has the following fields:
```
x::Float64
y::Float64			 
heading::Float64	# east of north (degrees)
max_step::Float64	# max distance vehicle can go per unit time (meters)
sensor::Sensor
```

## Sensor
The abstract `Sensor` type describes the sensing model of the vehicle.
Originally, the only sensor type was bearing only, but this has been expanded to consider other sensing modalities.

#### BearingOnly

#### DirOmni


## Filters
A filter is something that maintains a belief over the search space and updates it given new observations and vehicle locations.

Note that each filter maintains a belief, which is a questionable design decision.
In reality, a belief is something separate, fed into a filter to be updated.
However, the belief representation (discrete, Gaussian, etc) depends heavily on the filtering being applied.
In short, it just seems easier to maintain a single filter type rather than worry about a separate belief.

#### Discrete Filter
The constructor for a discrete filter is
```
DF(m::SearchDomain, n::Int)
```
where `n` is the number of cells per side.

If you create a new `Sensor` subtype called `NewSensor`, you need to implement the following methods for the `DF` type to work:
```
obs2bin(o::Float64, f::Filter, s::NewSensor)
O(x::Vehicle, s::NewSensor, theta::LocTuple, ob::Int, f::Filter)
```
The function `obs2bin` converts the continuous domain observation `o` into a discrete value, using the `num_bins` property of the discrete filter.
The binned observation `ob` is fed into `O`, which provides the probability of receiving `ob`.
I've implicitly assumed that the observations are discrete in the discrete filter, but I think only the search domain needs to be discretized, technically.

#### Extended Kalman Fiter
```
EKF(m::SearchDomain)
```

#### Unscented Kalman Fiter
```
UKF(m::SearchDomain)
```

#### Gaussian Fiter
The `GaussianFilter` abstract type is a child of `AbstractFilter` and a parent of `EKF` and `UKF`. I've thought about calling this `KalmanFilter` instead, but that could be ambiguous---someone could think this refers to a specific KF, rather than an abstract type. 

The `GaussianFilter` abstract type covers utilities that both `EKF` and `UKF` use.
The most important of these is the `Initializer` abstract type.
Each `EKF` and `UKF` instance contains an `Initializer` subtype that determines how the filter estimate should be initialized.

The default initializer is a `NaiveInitializer` sets the estimate to be the center of the search domain and uses a large initial covariance.

Another initializer is the `LSInitializer`, or least squares initializer. After taking `min_obs_num` observations, this initializer sets the mean to the point in the search domain yielding the smallest sum of least square differences between observed and expected observations. The code below shows how to initialize an instance of `LSInitializer` and modify some of its important fields:
```
lsi = LSInitializer(m::SearchDomain)
lsi.Sigma = 1e3*eye(2)
lsi.min_obs_num = 5
```


#### Particle Filter
To create a particle filter, you must provide the search domain `m` and desired number of particles `n`:
```
PF(m::SearchDomain, n::Int)
```
If you create a new `Sensor` subtype called `NewSensor`, you must implement the following function:
```
O(x::Vehicle, s::NewSensor, theta::LocTuple, o::Float64)
```
Currently, it is only required that this return a probability density, rather than a true probability.

#### Custom Filters
The code below is a template for creating your own filter type.
You must extend the `AbstractFilter` type and implement the following functions.
```
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
```

## Policies

#### RandomPolicy
A `RandomPolicy` simply moves the vehicle in a random direction.
```
RandomPolicy()
```

#### GreedyPolicy
A `GreedyPolicy` moves the agent in the direction that minimizes the expected entropy after moving.
```
GreedyPolicy(x::Vehicle, n::Int)
```
The integer `n` denotes how many actions should be considered.
If `n=6`, then the agent considers the expected entropy given 6 different directions, spaced an even 60 degrees apart.

#### CirclePolicy
A `CirclePolicy` moves the agent perpendicularly to the last recorded bearing measurement, which ends up drawing a circle around the source.
The constructor is as follows:
```
CirclePolicy()
```
The `CirclePolicy` implicitly assumes that the sensor is of `BearingOnly` type.

#### Custom Policy
You can create your own policies by extending the abstract `Policy` class and implementing the `action` function. Below is an example:
```
type CustomPolicy <: Policy
end

function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::CustomPolicy)
	# your policy code
	# must return action (2-tuple of Float64s)
end
```
Feel free to take advantage of the `normalize` function to ensure your action's norm is equal to the maximum distance the vehicle can take per time step:
```
normalize(a::Action, x::Vehicle)
```

## Simulations
```
steps!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int)
```
Alternatively, you can just call `steps!()`.
This will run a simulation for 10 steps. 
It assumes you have a model `m`, a Vehicle `x`, a filter `f`, and a policy `p` in the main module (with those names).
If you've named your objects that way, you don't have to remember the arguments required for `steps!`.
Alternatively, you can specify a number of steps: `steps!(30)`.


## Bearing Only Examples
The code below creates a search domain 100 meters by 100 meters, with a jammer located at (40,70).
The vehicle starts at (50,50) and moves at the default 4 m/s.
It is equipped with the standard `BearingOnly` sensor with 10 degree standard deviation.
A discrete filter with 100 bins is used for state estimation.
A `CirclePolicy` moves the UAV perpendicular to the last received bearing.
Measurements are received every half second.
```
using FEBOL

m = SearchDomain(100, 40, 70)
f = DF(m, 100)
x = Vehicle(50, 50)
p = CirclePolicy()

gif(m,x,f,p,18)
```
The resulting trajectory is saved as a gif. The jammer is a red triangle.
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif3.gif"/>
</p>
Alternatively, you can use a greedy entropy minimizing policy that considers 8 different directions (45 degrees apart).
```
using FEBOL

m = SearchDomain(100, 40, 70)
f = DF(m, 100)
x = Vehicle(50, 50)
p = GreedyPolicy(x, 8)

gif(m,x,f,p,18)
```
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif5.gif"/>
</p>
The following example is a particle filter with 10,000 particles. The green x marks the mean of the particles.
```
using FEBOL

m = SearchDomain(100, 40, 70)
f = PF(m, 10000)
x = Vehicle(50, 50)
p = CirclePolicy()

gif(m,x,f,p,18)
```
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif4.gif"/>
</p>

## Directional+Omni Examples
```
using FEBOL

m = SearchDomain(100, 40, 70)
f = DF(m, 100)
sensor = DirOmni("norm360.csv")
sensor.stds *= 0.2
x = Vehicle(50, 50, sensor)
p = GreedyPolicy(x, 8)

gif(m,x,f,p,17)
```
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif2.gif"/>
</p>
This example is also interesting:
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif1.gif"/>
</p>

## FOV examples
```
using FEBOL

m = SearchDomain(100, 40, 70)
f = DF(m, 100)
f.bin_range = 0:1
region_probs = [(60.0,0.9), 70.0,0.5), (180.0,0.1)]
sensor = FOV(region_probs)
x = Vehicle(50, 50, sensor)
p = GreedyPolicy(x, 8)

gif(m,x,f,p,18)
```
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif8.gif"/>
</p>

## TODO:

* handle observation directly over jammer
* implement particle filters
* implement centroid, variance functions
* rigorously test centroid functions
* allow user to specify degree discretization in discrete filter (also particle filter?)
* allow interpolations in plotting (instead of "none")

[![Build Status](https://travis-ci.org/dressel/FEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/FEBOL.jl)
