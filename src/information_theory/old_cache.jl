######################################################################
# old_cache.jl
#
# like infotheoretic.jl, but allows caching
#
# how this used to be implemented
######################################################################


export make_cache
function make_cache(df::DF)
    cache = zeros(df.n, df.n, df.n, df.n, length(df.obs_list))
    for xi = 1:df.n, yi = 1:df.n
        x = (xi-0.5) * df.cell_size
        y = (yi-0.5) * df.cell_size
        xp = (x,y,0.0)
        for txi = 1:df.n, tyi = 1:df.n
            tx = (txi-0.5) * df.cell_size
            ty = (tyi-0.5) * df.cell_size
            theta = (tx,ty,0.0,0.0)
            for (oi,o) in enumerate(df.obs_list)
                cache[xi,yi,txi,tyi,oi] = O(df.sensor, theta, xp, o)
            end
        end
    end
    return cache
end


function mutual_information(df::DF, cache::Array{Float64,5})
    mut_info = zeros(df.n, df.n)
    for xi = 1:df.n, yi = 1:df.n
        # TODO: really, should have some other heading
        xp = (xi, yi, 0)
        mut_info[xi,yi] = mutual_information(df, xp, cache)
    end
    return mut_info
end

function mutual_information(df::DF, xp::NTuple{3,Int}, cache::Array{Float64,5})
	H_o = 0.0
	H_o_t = 0.0

    for (oi,o) in enumerate(df.obs_list)
		po = p_obs(df, xp, oi, cache)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over possible jammer locations
		for txi = 1:df.n, tyi = 1:df.n
            if df.b[txi, tyi] > 0.0
                pot = cache[ xp[1], xp[2], txi, tyi, oi ]
                if pot > 0.0
                    H_o_t -= pot * df.b[txi, tyi] * log(pot)
                end
            end
		end
	end
	return H_o - H_o_t
end

function p_obs(df::DF, xp::NTuple{3,Int}, oi::Int, cache::Array{Float64,5})
    prob = 0.0
    for txi = 1:df.n, tyi = 1:df.n
        if df.b[txi,tyi] > 0.0
            prob += df.b[txi, tyi] * cache[ xp[1], xp[2], txi, tyi, oi ]
        end
    end
    return prob
end
