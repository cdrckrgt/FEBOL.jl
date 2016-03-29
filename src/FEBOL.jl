module FEBOL

using Distributions: Normal, cdf
using PyPlot: imshow,plot,xlabel,ylabel,contour,figure,pause,hold,axis

export SearchDomain
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF
export initial_belief
export Vehicle
export update!
export plot_b

typealias Obs Int64

# Assume these are square
type SearchDomain
	length::Float64
	theta::NTuple{2,Float64}

	function SearchDomain(length::Float64,theta_x::Float64,theta_y::Float64)
		@assert theta_x <= length
		@assert theta_y <= length
		return new(length, (theta_x, theta_y))
	end
end

include("vehicle.jl")
include("observations.jl")
include("beliefs.jl")
include("discrete.jl")
include("plotting.jl")

end # module
