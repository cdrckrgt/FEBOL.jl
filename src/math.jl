######################################################################
# includes stuff I don't want to need distributions for...
######################################################################


# only handles stuff in 2d
function my_pdf(x::Vector{Float64}, m::Vector{Float64}, S::Matrix{Float64})
	return my_pdf((x[1],x[2]), m, S)
end

function my_pdf(x::NTuple{2,Float64},m::Vector{Float64},S::Matrix{Float64})
	dx = x[1] - m[1]
	dy = x[2] - m[2]

	Si = inv(S)
	e_stuff = dx*Si[1,1]*dx + dx*Si[1,2]*dy + dy*Si[2,1]*dx + dy*Si[2,2]*dy
	return det(sqrtm(Si)) * exp(-0.5 * e_stuff) / (2.0 * pi)
end

function my_cdf(sigma::Float64, x::Float64)
	temp = x / (sigma*sqrt(2.0))
	return 0.5 * (1. + erf(temp))
end

# Fits an angle into -180 to 180
# Assume the angle is within -360 to 360
function fit_180(angle::Float64)
	if angle > 180.0
		#angle = 360.0 - angle   # omfg this is how i did it before. wrong.
		angle -= 360.0
	elseif angle < -180.0
		angle += 360.0
	end
	return angle
end


######################################################################
# Find the true angle between UAV and jammer.
#
# Parameters:
#  xp - location of vehicle
#  theta - location of jammer
#
# Returns true angle, measured from north, in degrees.
function true_bearing(xp::NTuple{2,Float64}, theta)
	xr = theta[1] - xp[1]
	yr = theta[2] - xp[2]
	return mod(rad2deg(atan2(xr,yr)), 360)
end
true_bearing(p::Pose, theta) = true_bearing( (p[1], p[2]), theta)
function true_bearing(xp, theta::Vector{Float64})
	return true_bearing(xp, (theta[1], theta[2]))
end
function true_bearing(xp::Pose, theta::Vector{Float64})
	return true_bearing((xp[1], xp[2]), (theta[1], theta[2]))
end

export get_distance
function get_distance(p1, p2)
    dx = p1[1] - p2[1]
    dy = p1[2] - p2[2]

    return sqrt(dx*dx + dy*dy)
end
function get_distance2(p1, p2)
    dx = p1[1] - p2[1]
    dy = p1[2] - p2[2]

    return dx*dx + dy*dy
end


######################################################################
# I don't remember what these are for at all
function angle_mean(v)
	n = length(v)
	sin_sum = 0.0
	cos_sum = 0.0
	for i = 1:n
		sin_sum += sin(v[i])
		cos_sum += cos(v[i])
	end
	return mod(rad2deg(atan2(sin_sum, cos_sum)), 360.)
end

function angle_mean(v,w)
	n = length(v)
	sin_sum = 0.0
	cos_sum = 0.0
	for i = 1:n
		sin_sum += w[i] * sin(v[i])
		cos_sum += w[i] * cos(v[i])
	end
	return mod(rad2deg(atan2(sin_sum, cos_sum)), 360.)
end


######################################################################
# I don't really remember using this
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
