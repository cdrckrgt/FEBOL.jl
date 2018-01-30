function parsim(m::SearchDomain, vsu, n_sims::Int)
    np = nprocs()       # number of processes available
    i = 1
    nextidx() = (idx=i; i+=1; idx)
    costs = zeros(n_sims, length(vsu))
    @sync begin
        for p=1:np
            if p != myid() || np == 1
                @async begin
                    while true
                        idx = nextidx()
                        idx > n_sims && break
                        costs[idx,:]=remotecall_fetch(psim, p, idx, m, vsu)
                    end
                end
            end
        end
    end
    return costs
end

# like simulation, but special for parallel simulations
# main change is deep-copying of arguments so as to not disturb them
function psim(idx::Int, m::SearchDomain, vsu::Vector{SimUnit})

    # copy these so they don't get messed up by other cores
    m = deepcopy(m)
    vsu = deepcopy(vsu)

    # change the target location
    theta!(m)

    costs = zeros(length(vsu))

    print("Simulation ", idx, ": ")
    for (su_ind, su) in enumerate(vsu)
        print(su_ind, ",")

        # reset the filter, vehicle, and policy
        reset!(m, su)

        costs[su_ind] = simulate(m, su)
    end
    println("complete.")
    return costs
end
