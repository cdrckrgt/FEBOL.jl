######################################################################
# waypoint.jl
######################################################################

# TODO: figure out export strategy
export WaypointPolicy

# to simplify some notation in this file
#typealias VVF Vector{Vector{Float64}}
const VVF = Vector{Vector{Float64}}

type WaypointPolicy <: Policy
	current_index::Int
	waypoints::VVF

	function WaypointPolicy(waypoints::VVF)
		new(2, waypoints)
	end
end

# TODO: actually require the third index
function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::WaypointPolicy)
	dx = p.waypoints[p.current_index][1] - x.x
	dy = p.waypoints[p.current_index][2] - x.y
	p.current_index += 1

	# Just wrap around? Or maybe we should stay still?
	if p.current_index > length(p.waypoints)
		p.current_index = 1
	end

	return dx, dy, 0.0
end
