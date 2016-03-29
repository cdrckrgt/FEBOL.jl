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

## TODO:

* handle observation directly over jammer
* implement particle filters
* implement centroid, variance functions
* draw uav instead of star
* allow interpolations in plotting (instead of "none")

[![Build Status](https://travis-ci.org/dressel/FEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/FEBOL.jl)
