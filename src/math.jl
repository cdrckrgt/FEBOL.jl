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
