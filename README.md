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
A `GreedyPolicy` moves
```
GreedyPolicy(x::Vehicle, n::Int)
```

#### OrthoPolicy
The constructor is as follows:
```
OrthoPolicy()
```

#### Custom Policy
You can create your own policies by extending the abstract `Policy` class and implementing the `action` function.
To implement a policy named `PolicyName`
```
type CustomPolicy <: Policy
end

function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::CustomPolicy)
	# your policy code
	# must return action (2-tuple of Float64s)
end
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
