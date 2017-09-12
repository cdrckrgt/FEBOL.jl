module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using PyCall
using PyPlot: imshow,xlabel,ylabel,contour,figure,pause,hold,axis, title, scatter, gcf, savefig, matplotlib, rc, tick_params, clf, cla
import PyPlot.plot
using StatsBase: sample
using Reel


export SearchDomain, theta!
export Vehicle
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF, EKF, EIF, UKF, PF
export update!, centroid, covariance, entropy, reset!
export observe
export plot, hold, plot_vehicle, title, pause
export step!, steps!, batchsim, batchsim2, sim
export Policy, RandomPolicy, SitPolicy, GreedyPolicy, CirclePolicy, SpinPolicy, GaussianMPC, GreedyPolicy2
export action, act!
export makenorm
export Sensor, RangeOnly, BearingOnly, DirOmni, FOV
export gif
export print_belief
export LSInitializer, NaiveInitializer
export my_pdf
export true_bearing

# Stuff for ergodicity
export ErgodicManager, phik!, reconstruct, decompose, ergodic_score
export TrajectoryManager, create_trajectory

# Old (Julia v0.5) way of doing it
#typealias Pose		    NTuple{3, Float64}    # x, y, heading
#typealias Action        NTuple{3, Float64}    # dx, dy, dh
#typealias LocTuple      NTuple{2, Float64}    # x, y
#typealias ObsBin        Int64

const Pose = NTuple{3, Float64}
const Action = NTuple{3, Float64}
const LocTuple = NTuple{2, Float64}
const ObsBin = Int64

export Pose

# math functions
include("math.jl")

# search domain
include("searchdomain.jl")

# Sensing models
include("sensors/sensors.jl")

include("vehicle.jl")

# Filters
include("filters/filters.jl")

# policies
include("infotheoretic.jl")
include("policies/policies.jl")
#include("policy.jl")

# Simulation and Plotting
include("simulations/termination.jl")
include("simulations/costs.jl")
include("simulations/simulations.jl")
include("simulations/plotting.jl")
#PyCall.PyDict(matplotlib["rcParams"])["font.family"]=["Times New Roman"]
#PyCall.PyDict(matplotlib["rcParams"])["font.family"]=["serif"]
rc("font", family="serif")
include("simulations/gif.jl")
include("simulations/batchsim.jl")

# for some gps stuff...
# TODO: let's talk about this gps stuff in README
include("gps.jl")

# for some ergodicity stuff...
# 1/23/2017 commenting this out because I don't think I use it much
#include("ergodicity/ergodicity.jl")
#include("ergodicity/trajectory.jl")
#include("ergodicity/max_trajectory.jl")

end # module
