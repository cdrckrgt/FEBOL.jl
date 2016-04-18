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


## Example
The code below creates a search domain 100 meters by 100 meters, with a jammer located at (5,85).
A discrete filter with 50 bins is used for the belief representation.
A vehicle starts at (50,50) and uses a greedy entropy-minimizing policy with 16 different directions.
```
m = SearchDomain(100, 5, 85)
f = DF(m, 50)
x = Vehicle(50, 50)
p = GreedyPolicy(x, 16)
```

## Another Example
<p align="center">
<img src="http://stanford.edu/~dressel/gifs/gif1.gif"/>
</p>

## TODO:

* handle observation directly over jammer
* implement particle filters
* implement centroid, variance functions
* rigorously test centroid functions
* allow user to specify degree discretization in discrete filter (also particle filter?)
* draw uav instead of star
* allow interpolations in plotting (instead of "none")

[![Build Status](https://travis-ci.org/dressel/FEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/FEBOL.jl)
