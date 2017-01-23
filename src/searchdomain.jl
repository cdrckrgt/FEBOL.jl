######################################################################
# searchdomain.jl
# Handles the type and functions required for the search domain.kl;w
######################################################################
"""
type `SearchDomain`

Constructors:

`SearchDomain(length, theta_x, theta_y)`

Makes a square search domain with a side-length of `length`. The RF source is located at `(theta_x, theta_y)`.

`SearchDomain(length)`

Assumes a random location to the RF source (theta).
"""
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
	function SearchDomain(length::Real)
		length = float(length)
		theta_x = length * rand()
		theta_y = length * rand()
		return new(length, (theta_x, theta_y))
	end
end

"""
`theta!(m::SearchDomain, xj::Real, yj::Real)`

`theta!(m::SearchDomain, theta::LocTuple)`

`theta!(m::SearchDomain)`

Change the jammer location of the search domain.

If no location is provided, a random one is selected.
"""
theta!(m::SearchDomain) = theta!(m, m.length * rand(), m.length * rand())

function theta!(m::SearchDomain, xj::Real, yj::Real)
	theta!( m, (float(xj), float(yj)) )
end
function theta!(m::SearchDomain, theta::LocTuple)
	@assert theta[1] <= m.length
	@assert theta[2] <= m.length
	m.theta = theta
end
