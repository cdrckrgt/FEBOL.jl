# FEBOL

## SearchDomain
To create a 10m by 10m search domain with a jammer located at 9.5, 9.5 you can do the following:
```
m = SearchDomain(10.0, 9.5, 9.5)
```
The jammer must be within 0 and the length of one side of the domain.

## Filters

### Discrete Filters
The constructor for a discrete filter is
```
DF(m::SearchDomain, n::Int)
```
where `n` is the number of cells per side.

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
If `n=6`, then the agent considers the expected entropy given 6 different directions, spaced and even 60 degrees apart.

#### OrthoPolicy
The constructor is as follows:
```
OrthoPolicy()
```

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

## TODO:

* handle observation directly over jammer
* implement particle filters
* implement centroid, variance functions
* rigorously test centroid functions
* allow user to specify degree discretization in discrete filter (also particle filter?)
* draw uav instead of star
* allow interpolations in plotting (instead of "none")

[![Build Status](https://travis-ci.org/dressel/FEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/FEBOL.jl)
