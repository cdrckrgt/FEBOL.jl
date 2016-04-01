module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using PyPlot: imshow,xlabel,ylabel,contour,figure,pause,hold,axis, title
import PyPlot.plot

export SearchDomain, theta!
export Vehicle
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF, EKF, EIF
export update!, centroid, entropy, reset!
export observe
export plot
export step!, steps!
export Policy, RandomPolicy, GreedyPolicy, OrthoPolicy

typealias Action   NTuple{2, Float64}
typealias LocTuple NTuple{2, Float64}
typealias Obs      Int64

# Assume these are square

include("searchdomain.jl")
include("vehicle.jl")
include("observations.jl")

# Filters
include("filters.jl")

# policies
include("infotheoretic.jl")
include("policy.jl")

# Simulation and Plotting
include("simulations.jl")
include("plotting.jl")

end # module
