######################################################################
# filters.jl
# Handles the different filters
######################################################################

abstract AbstractFilter

# Update function is a Bayesian update
update!(f::AbstractFilter, x::Vehicle, o::Int) = update!(f, x, float(o))
function update!(f::AbstractFilter, x::Vehicle, o::Float64)
	error(typeof(f), " does not yet implement update!(f,x,o).")
end

function centroid(f::AbstractFilter)
	error(typeof(f), " does not yet implement centroid(f).")
end

include("observations.jl")		# helper needed for discrete. ugly
include("df.jl")
include("ekf.jl")
include("eif.jl")
