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

function steps!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_steps::Int; video::Bool=true)
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
function steps!(num_steps::Int64; video::Bool=true)
	steps!(Main.m, Main.x, Main.f, Main.p, num_steps, video=video)
end

# Really, we want a vector of AbstractFilters
# TODO: vector of abstract filters
"""
`batchsim(m,x,f,p,num_sims)`

Currently:

* starts vehicle at center of search domain
* runs each simulation for 40 steps
* results is num_sims by num_filters array

"""
function batchsim(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy, num_sims::Int)
	batchsim(m, x, [f], p, num_sims)
end
function batchsim{TF<:AbstractFilter}(m::SearchDomain, x::Vehicle, farr::Vector{TF}, p::Policy, num_sims::Int, num_steps::Int)
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
			print("f:", fi)
			f = farr[fi]
			x.x = m.length / 2.0
			x.y = m.length / 2.0
			steps!(m, x, f, p, num_steps; video=false)
			print("d ")

			# calculate the error
			c = centroid(f)
			results[i, fi] = norm2(c, m.theta)
			reset!(f)
		end
	end
	return results
end
# This is for testing with a dumb policy that we can apply to all filters
function batchsim2{TF<:AbstractFilter}(m::SearchDomain, x::Vehicle, farr::Vector{TF}, p::Policy, num_sims::Int, num_steps::Int)
	num_filters = length(farr)
	results = zeros(num_sims, num_filters)
	f1 = farr[1]
	for i = 1:num_sims
		println()
		print("Starting sims ", i, ": ")
		# Set new jammer location
		jx = m.length * rand()
		jy = m.length * rand()
		theta!(m, jx, jy)
		# Reset vehicle to zero
		x.x = m.length / 2.0
		x.y = m.length / 2.0

		# n is total steps per simulation
		for j = 1:num_steps
			# receive an observation
			o = observe(m,x)

			# update belief of all filters
			#for fi = 1:num_filters
			#	f = farr[fi]
			#	update!(f, x, o)
			#end
			for f in farr
				update!(f, x, o)
			end
			a = action(m, x, o, f1, p)
			act!(m,x,a)

		end
		# compute the centroid and results
		for fi = 1:num_filters
			f = farr[fi]
			c = centroid(f)
			results[i, fi] = norm2(c, m.theta)
			reset!(f)
		end
	end
	return results
end

function norm2(x::LocTuple, y::LocTuple)
	d1 = x[1] - y[1]
	d2 = x[2] - y[2]
	return sqrt(d1*d1 + d2*d2)
end
