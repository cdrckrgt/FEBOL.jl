module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using PyPlot: imshow,xlabel,ylabel,contour,figure,pause,hold,axis
import PyPlot.plot

export SearchDomain
export Belief, Gaussian, ParticleSet, DiscreteBelief
export DF, EKF
export Vehicle
export update!
export centroid
export plot
export InfoMatrix, EIF

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

# Filters
include("filters.jl")
#include("df.jl")
#include("ekf.jl")
#include("eif.jl")

# Plotting
include("plotting.jl")

end # module
