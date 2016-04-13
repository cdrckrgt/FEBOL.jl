######################################################################
# sensors.jl
######################################################################

abstract Sensor

type BearingOnly <: Sensor
	noise_sigma::Float64
end

type DirOmni <: Sensor
	means::Vector{Float64}
	stds::Vector{Float64}
end
