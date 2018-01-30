# FEBOL

[![Build Status](https://travis-ci.org/dressel/FEBOL.jl.svg?branch=master)](https://travis-ci.org/dressel/FEBOL.jl)
[[![Coverage Status](https://coveralls.io/repos/github/dressel/FEBOL.jl/badge.svg?branch=master)](https://coveralls.io/github/dressel/FEBOL.jl?branch=master)
![Docs](https://img.shields.io/badge/docs-latest-blue.svg)](https://feboljl.readthedocs.io/en/latest/)

This package was originally made to test various filters in the bearing-only localization problem (the name FEBOL comes from Filter Exploration for Bearing-Only Localization).
However, it has since expanded into a general framework for simulating UAV-based localization of a single, stationary target.
The user can choose from a variety of sensors, filters, and policies (or make his own). 

Visualizing simulations requires the package [FEBOLPlots.jl](https://github.com/dressel/FEBOLPlots.jl).
Visualizations are done in PyPlot, but PyPlot annoyingly takes about 5 seconds to load and might not be available on all machines (like a lab server).
Therefore, visualization code is kept in FEBOLPlots.jl.

Check out the [complete documentation](http://feboljl.readthedocs.io/en/latest/index.html).

## Installation


```julia
Pkg.clone("https://github.com/dressel/FEBOL.jl.git")
```

To get visualization functionality, you also need FEBOLPlots.jl.

```julia
Pkg.clone("https://github.com/dressel/FEBOLPlots.jl.git")
```

## Quick Example


```julia
using FEBOL

m = SearchDomain(200, 140, 170)
s = BearingOnly(5)
f = DF(m, 41, s, 0:10:350)
x = Vehicle(100, 100, 0, 5, s)
p = GreedyPolicy(x, 4)

using FEBOLPlots
visualize(m, x, f, p)
```

There is much more functionality; check out the [complete documentation](http://feboljl.readthedocs.io/en/latest/index.html) for more.

## TODO:

* interface with ParticleFilters.jl
* allow user to specify degree discretization in discrete filter (also particle filter?)
* allow interpolations in plotting (instead of "none")
