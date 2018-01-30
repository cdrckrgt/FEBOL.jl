######################################################################
# batchsim.jl
#
# running many simulations
######################################################################

function simulate(m::SearchDomain, suv::Vector{SimUnit}, n_sims::Int)

    costs = zeros(n_sims, length(suv))
    for sim_ind = 1:n_sims
        theta!(m)
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

function batchsim(m::SearchDomain, uav_array::Vector{SimUnit}, n_sims::Int)

    costs = zeros(n_sims, length(uav_array))
    for sim_ind = 1:n_sims
        theta!(m)
        print("Simulation ", sim_ind, ": ")
        for (uav_ind, uav) in enumerate(uav_array)
            print(uav_ind, ",")

            # reset the filter, vehicle, and policy
            reset!(m, uav)

            # before doing anything else, we observe
            #  and update filter once
            o = observe(m, uav.x)
            update!(uav, o)

            # This was our first step; steps count number of observations
            step_count = 1

            # What was the cost to getting this first observation?
            temp_cost = get_cost(uav, m)

            while !is_complete(uav.f, uav.tc, step_count)
                # act
                a = action(m, uav, o)
                act!(m, uav.x, a)

                # get cost and update step count
                temp_cost += get_cost(uav, m, a)
                step_count += 1

                # observe and update
                o = observe(m, uav.x)
                update!(uav, o)
            end

            costs[sim_ind, uav_ind] = temp_cost
        end
        println("complete.")
    end
    return costs
end
