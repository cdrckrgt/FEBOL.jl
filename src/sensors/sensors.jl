######################################################################
# sensors.jl
######################################################################

abstract Sensor

include("bearing.jl")
include("range.jl")
include("diromni.jl")
include("fov.jl")

function observe(m::SearchDomain, s::Sensor, p::Pose)
	error("`observe` not defined for this sensor!")
end

# Required for particle filter
# does not require probability, returns density
O(s::Sensor, theta, p::Pose, o::Float64)


# Takes a vector of 36 angles and turns it into 360 via linear interpolation
function makenorm()
	data = readcsv("norm.csv")
	values = vec(data[:,2])
	new_values = zeros(360)
	for i = 1:35
		val_start = 10*(i-1)+1
		new_values[val_start] = values[i]
		for j = 1:9
			new_values[val_start + j] = (10.0-j)*values[i] + j*values[i+1]
			new_values[val_start + j] /= 10.0
		end
	end
	# now handle the last one
	i = 36
	val_start = 10*(i-1)+1
	new_values[val_start] = values[i]
	for j = 1:9
		new_values[val_start + j] = (10.0-j)*values[i] + j*values[1]
		new_values[val_start + j] /= 10.0
	end
	return new_values
end
