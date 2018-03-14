######################################################################
# cache2.jl
#
# like infotheoretic.jl, but allows caching
######################################################################


export make_cache
export make_cache2

function make_cache(df::DF)
    n  = 2*df.n - 1
    cache = zeros(n, n, length(df.obs_list))

    for dxi = 1:n, dyi = 1:n
        dx = dxi - df.n
        dy = dyi - df.n

        # generate a theta, xp that fit this
        # just put vehicle at zero?
        xp = (0.0,0.0,0.0)
        tx = dx * df.cell_size
        ty = dy * df.cell_size
        theta = (tx, ty, 0.0, 0.0)

        for (oi,o) in enumerate(df.obs_list)
            cache[dxi, dyi, oi] = O(df.sensor, theta, xp, o)
        end
    end

    return cache
end




function mutual_information(df::DF, cache::Array{Float64,3})
    mut_info = zeros(df.n, df.n)
    for xi = 1:df.n, yi = 1:df.n
        # TODO: really, should have some other heading
        xp = (xi, yi, 0)
        mut_info[xi,yi] = mutual_information(df, xp, cache)
    end
    return mut_info
end

function mutual_information(df::DF, xp::NTuple{3,Int}, cache::Array{Float64,3})
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
                #pot = cache[ xp[1], xp[2], txi, tyi, oi ]
                pot = cache[txi-xp[1]+df.n, tyi-xp[2]+df.n, oi]
                if pot > 0.0
                    H_o_t -= pot * df.b[txi, tyi] * log(pot)
                end
            end
		end
	end
	return H_o - H_o_t
end

# for the R2 case
function p_obs(df::DF, xp::NTuple{3,Int}, oi::Int, cache::Array{Float64,3})
    prob = 0.0
    for txi = 1:df.n, tyi = 1:df.n
        if df.b[txi,tyi] > 0.0
            prob += df.b[txi, tyi]*cache[txi-xp[1]+df.n, tyi-xp[2]+df.n, oi]
        end
    end
    return prob
end

######################################################################
# SE(2)
######################################################################

function make_cache2(df::DF)
    n  = 2*df.n - 1
    cache = zeros(n, n, 36, length(df.obs_list))

    for dxi = 1:n, dyi = 1:n
        dx = dxi - df.n
        dy = dyi - df.n

        for hi = 1:36

            h = (hi - 1) * 10.0
            # generate a theta, xp that fit this
            # just put vehicle at zero?
            xp = (0.0,0.0,h)
            tx = dx * df.cell_size
            ty = dy * df.cell_size
            theta = (tx, ty, 0.0, 0.0)

            for (oi,o) in enumerate(df.obs_list)
                cache[dxi, dyi, hi, oi] = O(df.sensor, theta, xp, o)
            end
        end
    end

    return cache
end

function mutual_information(df::DF, cache::Array{Float64,4})
    mut_info = zeros(df.n, df.n, 36)
    for xi = 1:df.n, yi = 1:df.n, hi = 1:36
        xp = (xi, yi, hi)
        mut_info[xi,yi, hi] = mutual_information(df, xp, cache)
    end
    return mut_info
end

function mutual_information(df::DF, xp::NTuple{3,Int}, cache::Array{Float64,4})
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
                #pot = cache[ xp[1], xp[2], txi, tyi, oi ]
                pot = cache[txi-xp[1]+df.n, tyi-xp[2]+df.n, xp[3], oi]
                if pot > 0.0
                    H_o_t -= pot * df.b[txi, tyi] * log(pot)
                end
            end
		end
	end
	return H_o - H_o_t
end

function p_obs(df::DF, xp::NTuple{3,Int}, oi::Int, cache::Array{Float64,4})
    prob = 0.0
    for txi = 1:df.n, tyi = 1:df.n
        if df.b[txi,tyi] > 0.0
            prob += df.b[txi, tyi]*cache[txi-xp[1]+df.n, tyi-xp[2]+df.n, xp[3], oi]
        end
    end
    return prob
end
