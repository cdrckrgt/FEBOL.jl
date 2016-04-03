######################################################################
# simulations.jl
######################################################################

# sample an observation
#  1. receives an observation
#  2. updates belief (contained in filter)
#  3. updates vehicle position according to some policy
# For right now, assume a random policy
function step!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy; video::Bool=true)

	# receive an observation
	o = observe(m,x)

	# update belief
	update!(f, x, o)

	# update vehicle position
	a = action(m, x, o, f, p)
	act!(m,x,a)

	if video
		hold(false)
		plot(m, f, x)
	end
end

function steps!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int; video=true)
	# Show current step first
	if video
		hold(false)
		plot(m,f,x)
	end

	# Then go through steps
	for i = 1:num_steps
		if video; pause(.5); end
		step!(m,x,f,p; video=video)
		#title("e = $(round(entropy(f),2))")
	end
end

function steps!()
	steps!(10)
end
function steps!(num_steps::Int64)
	steps!(Main.m, Main.x, Main.f, Main.p, num_steps)
end

# Really, we want a vector of AbstractFilters
# TODO: vector of abstract filters
"""
`batchsim(m,x,f,p,num_sims)`

Currently:

* starts vehicle at center of search domain
* runs each simulation for 10 steps
* results is num_sims by num_filters array

"""
function batchsim(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_sims::Int)
	batchsim(m, x, [f], p, num_sims)
end
function batchsim{TF<:AbstractFilter}(m::SearchDomain, x::Vehicle, farr::Vector{TF}, p::Policy, num_sims::Int)
	num_filters = length(farr)
	results = zeros(num_sims, num_filters)
	for i = 1:num_sims
		println()
		print("Starting sims ", i, ": ")
		# Set new jammer location
		jx = m.length * rand()
		jy = m.length * rand()
		theta!(m, jx, jy)

		# run the simulation for all filters/policies
		for fi = 1:num_filters 
			print("f", fi, " ")
			f = farr[fi]
			x.x = m.length / 2.0
			x.y = m.length / 2.0
			steps!(m, x, f, p, 10; video=false)

			# calculate the error
			c = centroid(f)
			results[i, fi] = norm2(c, m.theta)
			reset!(f)
		end
	end
	return results
end

norm2(x::LocTuple, y::LocTuple) = sqrt( (x[1]-y[1])^2 + (x[2]-y[2])^2 )
