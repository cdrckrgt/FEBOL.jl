######################################################################
# random2.jl
# 
# random policy for bearing only method
######################################################################
type RandomPolicy2 <: Policy end
export RandomPolicy2
function action(m::SearchDomain, v::Vehicle, o::Float64, f::DF, p::RandomPolicy2)
	xp = rand() * m.length
	yp = rand() * m.length
	return xp - v.x, yp - v.y, 0.0
end
