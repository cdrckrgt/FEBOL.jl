module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using PyPlot: imshow,xlabel,ylabel,contour,figure,pause,hold,axis, title, scatter, gcf
import PyPlot.plot
using StatsBase: sample, WeightVec
using Reel

export SearchDomain, theta!
export Vehicle
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF, EKF, EIF, PF
export update!, centroid, entropy, reset!
export observe
export plot
export step!, steps!, batchsim, batchsim2
export Policy, RandomPolicy, SitPolicy, GreedyPolicy, CirclePolicy, SpinPolicy
export action, act!
export makenorm
export Sensor, BearingOnly, DirOmni
export gif

typealias Pose		    NTuple{3, Float64}    # x, y, heading
typealias Action        NTuple{3, Float64}    # dx, dy, dh
typealias LocTuple      NTuple{2, Float64}    # x, y
typealias ObsBin        Int64

include("searchdomain.jl")

# Sensing models
include("sensors.jl")

include("vehicle.jl")
include("observations.jl")

# Filters
include("filters/filters.jl")

# policies
include("infotheoretic.jl")
include("policies/policies.jl")
#include("policy.jl")

# Simulation and Plotting
include("simulations.jl")
include("plotting.jl")
include("gif.jl")

end # module
