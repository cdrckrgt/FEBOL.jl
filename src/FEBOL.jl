module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using PyPlot: imshow,xlabel,ylabel,contour,figure,pause,hold,axis, title, scatter, gcf, savefig
import PyPlot.plot
using StatsBase: sample, WeightVec
using Reel

export SearchDomain, theta!
export Vehicle
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF, EKF, EIF, UKF, PF
export update!, centroid, covariance, entropy, reset!
export observe
export plot, hold
export step!, steps!, batchsim, batchsim2, sim
export Policy, RandomPolicy, SitPolicy, GreedyPolicy, CirclePolicy, SpinPolicy, GaussianMPC, GreedyPolicy2
export action, act!
export makenorm
export Sensor, BearingOnly, DirOmni, FOV
export gif
export print_belief
export LSInitializer, NaiveInitializer
export my_pdf
export true_bearing

# Stuff for ergodicity
export ErgodicManager, phik!, reconstruct, decompose, ergodic_score
export TrajectoryManager, create_trajectory

typealias Pose		    NTuple{3, Float64}    # x, y, heading
typealias Action        NTuple{3, Float64}    # dx, dy, dh
typealias LocTuple      NTuple{2, Float64}    # x, y
typealias ObsBin        Int64

export Pose

# math functions
include("math.jl")

# search domain
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

# for some gps stuff...
# TODO: let's talk about this gps stuff in README
include("gps.jl")

# for some ergodicity stuff...
include("ergodicity/ergodicity.jl")
include("ergodicity/trajectory.jl")
include("ergodicity/max_trajectory.jl")

end # module
