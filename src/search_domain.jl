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
mutable struct SearchDomain{M <: MotionModel}
    length::Float64
    theta::LocTuple
    motion_model::M
end
function SearchDomain(length::Real, theta::NTuple{2,Real}, mm::MotionModel=NoMotion())
    return SearchDomain(length, (theta[1], theta[2], 0.0, 0.0), mm)
end

function SearchDomain(length::Real, theta_x::Real, theta_y::Real, mm::MotionModel=NoMotion())
    length = float(length)
    theta_x = float(theta_x)
    theta_y = float(theta_y)
    @assert theta_x <= length
    @assert theta_y <= length
    return SearchDomain(length, (theta_x, theta_y, 0.0, 0.0), mm)
end
function SearchDomain(length::Real, mm::MotionModel=NoMotion())
    length = float(length)
    theta_x = length * rand()
    theta_y = length * rand()
    return SearchDomain(length, (theta_x, theta_y, 0.0, 0.0), mm)
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
    theta!( m, (float(xj), float(yj), 0.0, 0.0) )
end
function theta!(m::SearchDomain, theta::LocTuple)
    @assert theta[1] <= m.length
    @assert theta[2] <= m.length
    m.theta = theta
end

function move_target!(m::SearchDomain)
    m.theta = move_target(m.motion_model, m.theta, m.length)
end
