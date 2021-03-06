#####################################################################
# simulate.jl
#
# TODO: eliminate steps and make simulate with a simunit (maybe)
######################################################################

# sample an observation
#  1. receives an observation
#  2. updates belief (contained in filter)
#  3. updates vehicle position according to some policy
# For right now, assume a random policy
function step!(m::SearchDomain, x::Vehicle, f::AbstractFilter, p::Policy)
	# observe, update belief, select action, and act
	o = observe(m,x)
	update!(f, x, o)
	a = action(m, x, o, f, p)
	act!(m,x,a)
end

function simulate(m::SearchDomain,
                  x::Vehicle,
                  f::AbstractFilter,
                  p::Policy,
                  n_steps::Int=10
                 )

    tc = StepThreshold(n_steps)

	# Then go through steps
	step_count = 0
	while !is_complete(f, tc, step_count)
		step!(m,x,f,p)
		step_count += 1
	end
end

function simulate(m::SearchDomain, su::SimUnit)

    # Don't reset the filter, vehicle, and policy
    # I think I assume the SimUnit comes in clean and ready to go

    # What was the cost to getting this first observation?
    cost_sum = get_cost(su, m)

    # before doing anything else, we observe
    #  and update filter once
    o = observe(m, su.x)
    update!(su, o)

    # This was our first step; steps count number of observations
    step_count = 1

    while !is_complete(su.f, su.tc, step_count)
        # act
        a = action(m, su, o)
        act!(m, su.x, a)

        move_target!(m)

        # get cost and update step count
        cost_sum += get_cost(su, m, a)
        step_count += 1

        # observe and update
        o = observe(m, su.x)
        update!(su, o)
    end

    return cost_sum
end

# for running batches of simulations on a single sim unit
function simulate(m::SearchDomain, su::SimUnit, ns::Int)
    return vec( simulate(m, [su], ns) )
end

# for running batches of simulations on a vector of sim units
function simulate(m::SearchDomain, suv::Vector{SimUnit}, n_sims::Int; 
                  random_theta::Bool=true
                )

    theta_start = m.theta

    costs = zeros(n_sims, length(suv))
    for sim_ind = 1:n_sims

        # reset jammer to random location
        m.theta = theta_start
        if random_theta
            theta!(m)
        end

        print("Simulation ", sim_ind, ": ")
        for (su_ind, su) in enumerate(suv)
            print(su_ind, ",")

            # reset the filter, vehicle, and policy
            reset!(m, su)

            costs[sim_ind, su_ind] = simulate(m, su)
        end
        println("complete.")
    end
    return costs
end
