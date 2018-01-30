export parbatch
function parbatch(m::SearchDomain, vsu, n_sims::Int)
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
                        costs[idx,:]=remotecall_fetch(simulate,p,idx,m,vsu)
                    end
                end
            end
        end
    end
    return costs
end

function simulate(idx::Int, m::SearchDomain, vsu::Vector{SimUnit}; dc=true)

    if dc
        m = deepcopy(m)
        vsu = deepcopy(vsu)
    end

    costs = zeros(length(vsu))
    theta!(m)

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
