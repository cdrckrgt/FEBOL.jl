######################################################################
# simulations.jl
######################################################################

# sample an observation
#  1. updates vehicle position according to some policy
#  2. receives an observation
#  3. updates belief (contained in filter)
# For right now, assume a random policy
function step!(m::SearchDomain, x::Vehicle, f::AbstractFilter; video=false)

	# update vehicle position
	x.x += 1.0
	x.y += 1.0

	# receive an observation
	o = round(Int,observe(m, x))

	# update belief
	update!(f, x, o)

	if video
		close()
		plot(m, f, x)
	end
end
