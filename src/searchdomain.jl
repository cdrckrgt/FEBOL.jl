######################################################################
# searchdomain.jl
# Handles the type and functions required for the search domain.kl;w
######################################################################
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

"""
`theta!(m::SearchDomain, xj::Real, yj::Real)`

`theta!(m::SearchDomain, theta::LocTuple)`

Change the jammer location of the search domain.
"""
function theta!(m::SearchDomain, xj::Real, yj::Real)
	theta!( m, (float(xj), float(yj)) )
end
function theta!(m::SearchDomain, theta::LocTuple)
	@assert theta[1] <= m.length
	@assert theta[2] <= m.length
	m.theta = theta
end
