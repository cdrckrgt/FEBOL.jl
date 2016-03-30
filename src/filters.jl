######################################################################
# filters.jl
# Handles the different filters
######################################################################

abstract AbstractFilter

include("observations.jl")		# helper needed for discrete. ugly
include("df.jl")
include("ekf.jl")
include("eif.jl")
