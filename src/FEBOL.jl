module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using PyPlot: imshow,xlabel,ylabel,contour,figure,pause,hold,axis
import PyPlot.plot

export SearchDomain
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF, EKF
export Vehicle
export update!, centroid, entropy
export plot
export InfoMatrix, EIF
export step!, steps!
export Policy, RandomPolicy, GreedyPolicy, OrthoPolicy

typealias Action   NTuple{2, Float64}
typealias LocTuple NTuple{2, Float64}
typealias Obs      Int64

# Assume these are square
type SearchDomain
	length::Float64
	theta::LocTuple

	function SearchDomain(length::Float64,theta_x::Float64,theta_y::Float64)
		@assert theta_x <= length
		@assert theta_y <= length
		return new(length, (theta_x, theta_y))
	end
end

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
