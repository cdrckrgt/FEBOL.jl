######################################################################
# simulations.jl
######################################################################

# sample an observation
#  1. receives an observation
#  2. updates belief (contained in filter)
#  3. updates vehicle position according to some policy
# For right now, assume a random policy
function step!(m::SearchDomain, x::Vehicle, f::AbstractFilter; video=false)

	# receive an observation
	o = observe(m,x)

	# update belief
	update!(f, x, o)

	# update vehicle position
	p = RandomPolicy()
	p = GreedyPolicy(x, 8)

	a = action(m, x, f, p)
	act!(m,x,a)

	if video
		hold(false)
		plot(m, f, x)
	end
end

function steps!(m::SearchDomain, x::Vehicle, f::AbstractFilter, num_steps::Int)
	# Show current step first
	hold(false)
	plot(m,f,x)

	# Then go through steps
	for i = 1:num_steps
		pause(.5)
		step!(m,x,f; video=true)
	end
end

function steps!()
	steps!(10)
end
function steps!(num_steps::Int64)
	steps!(Main.m,Main.x,Main.f,num_steps)
end
