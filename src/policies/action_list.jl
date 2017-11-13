######################################################################
# action_list.jl
#
# for making lists of actions
######################################################################

export make_action_list

"""
`make_action_list(max_step, n_actions, headings; stay=true)`

`max_step` and `n_actions` determine the lateral movements

`headings::Vector{T<:Real}`
"""
function make_action_list(max_step, n_actions, headings; stay=true)

    # what we return
    actions = Action[]

    # loop over lateral actions
	angles = linspace(0.0, 360.0 - 360/n_actions, n_actions)
	for ang in angles
		ax = max_step * sind(ang)
		ay = max_step * cosd(ang)

        for h in headings
            push!(actions, (ax, ay, h*1.0))
        end
	end

    # include stay action
    if stay
        for h in headings
            push!(actions, (0.0, 0.0, h*1.0) )
        end
    end

    return actions
end
