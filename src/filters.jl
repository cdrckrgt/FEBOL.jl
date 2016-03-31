######################################################################
# filters.jl
# Handles the different filters
######################################################################

abstract AbstractFilter

"""
`update!(f::AbstractFilter, x::Vehicle, o::Real)`

Updates the belief of filter `f` given a vehicle `x` and observation `o`.
The observation `o` is a real number in [0,360).
If your filter requires this number to be binned into an integer, it must take care of that internally.
"""
update!(f::AbstractFilter, x::Vehicle, o::Int) = update!(f, x, float(o))
function update!(f::AbstractFilter, x::Vehicle, o::Float64)
	error(typeof(f), " does not yet implement update!(f,x,o).")
end


"""
`centroid(f::AbstractFilter)`

Returns the centroid of the filter's belief. If the belief is a Gaussian, this will return the mean.
"""
function centroid(f::AbstractFilter)
	error(typeof(f), " does not yet implement centroid(f).")
end



"""
`entropy(f::AbstractFilter)`

Returns the entropy of the filter's belief.
"""
function entropy(f::AbstractFilter)
	error(typeof(f), " does not yet implement entropy(f).")
end


# Include the different filters
include("observations.jl")		# helper needed for discrete. ugly
include("df.jl")
include("ekf.jl")
include("eif.jl")
