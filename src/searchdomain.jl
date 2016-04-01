type SearchDomain
	length::Float64
	theta::LocTuple

	function SearchDomain(length::Real, theta_x::Real, theta_y::Real)
		length = float(length)
		theta_x = float(theta_x)
		theta_y = float(theta_y)
		@assert theta_x <= length
		@assert theta_y <= length
		return new(length, (theta_x, theta_y))
	end
end
