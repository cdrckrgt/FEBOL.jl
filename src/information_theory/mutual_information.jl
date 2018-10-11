######################################################################
# mutual_information.jl
# has things like mutual information and stuff
######################################################################

export mutual_information3
export p_obs
# sum over all possible jammer locations
function p_obs(df::DF, xp::Pose, o)
    prob = 0.0
    for txi = 1:df.n, tyi = 1:df.n
        if df.b[txi,tyi] > 0.0
            tx = (txi-0.5) * df.cell_size
            ty = (tyi-0.5) * df.cell_size
            prob += df.b[txi, tyi] * O(df.sensor, (tx,ty,0.,0.), xp, o)
        end
    end
    return prob
end

function p_obs(pf::PF, xp::Pose, o)
    prob = 0.0
    for i = 1:pf.n
        prob += O(pf.model.sensor, particle(pf,i), xp, o)
    end
    return prob / pf.n
end

# TODO: I don't think I have to do it like this
function p_obs(s::Sensor, b::ParticleCollection, xp::Pose, o)
    prob = 0.0
    n = length(b.particles)
    for i = 1:n
        prob += O(s, particle(b,i), xp, o) * weight(b, i)
    end
    return prob / weight_sum(b)
end

#function mutual_information(pf::PF, xp::Pose)
#    H_o = 0.0
#    H_o_t = 0.0
#
#    # computing H_o
#    for o in pf.obs_list
#        po = p_obs(pf, xp, o)
#        if po != 0
#            H_o -= po * log(po)
#        end
#    end
#
#    # computing H_o_t
#    ws = weight_sum(pf.b)
#    for i in pf.n
#        weight(pf.b, i) / ws
#        for o in pf.obs_list
#            po = 
#            O(pf.model.sensor, pf.b.particles[i], xp, o)
#        end
#    end
#
#    return H_o - H_o_t
#end

# This is just a copy (and modification) of df's version
# The commented out version above is an attempt from scratch
function mutual_information(pf::PF, xp::Pose)
    return mutual_information(pf, pf._b, xp)
end
function mutual_information(pf::PF, b, xp::Pose)
	H_o = 0.0
	H_o_t = 0.0

    for o in pf.obs_list
		po = p_obs(pf, xp, o)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over possible jammer locations
        for i = 1:pf.n
            pot = O(pf.model.sensor, particle(pf,i), xp, o)
            if pot > 0.0
                H_o_t -= pot * log(pot) / pf.n
            end
        end
	end
	return H_o - H_o_t
end


# This is just a copy (and modification) of df's version
# The commented out version above is an attempt from scratch
function mutual_information(s::Sensor, b::ParticleCollection, xp::Pose, ol)
	H_o = 0.0
	H_o_t = 0.0

    for o in ol
		po = p_obs(s, b, xp, o)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over possible jammer locations
        ws = weight_sum(b)
        for i = 1:length(b.particles)
            pot = O(s, particle(b,i), xp, o)
            if pot > 0.0
                H_o_t -= pot * weight(b,i) * log(pot) / ws
            end
        end
	end
	return H_o - H_o_t
end

# computes mutual information for a specific vehicle location
# xp is a proposed pose
# really need to loop over all possible observations
export mutual_information
# this is the one that currently gets called with fov and greedy policy
function mutual_information(df::DF, xp::Pose)
	H_o = 0.0
	H_o_t = 0.0

    for o in df.obs_list
        po = 0.0        # probability of observation o

		# sum over possible jammer locations
		for txi = 1:df.n, tyi = 1:df.n
            tx = (txi - 0.5) * df.cell_size
            ty = (tyi - 0.5) * df.cell_size

            if df.b[txi, tyi] > 0.0
                pot = O(df.sensor, (tx,ty,0.,0.), xp, o)    # p(o | theta)
                po += df.b[txi, tyi] * pot
                if pot > 0.0
                    H_o_t -= pot * df.b[txi, tyi] * log(pot)
                end
            end
		end
		if po > 0.0
			H_o -= po * log(po)
		end
	end
	return H_o - H_o_t
end


# computes mutual information for all locations
# the provided discrete filter gives the domain size and belief
# I need a Vehicle or Sensor to pass into the observation function
function mutual_information(df::DF)
    mut_info = zeros(df.n, df.n)
    for xi = 1:df.n, yi = 1:df.n
        x = (xi - 0.5) * df.cell_size
        y = (yi - 0.5) * df.cell_size
        # TODO: really, should have some other heading
        xp = (x, y, 0.0)
        mut_info[xi,yi] = mutual_information(df, xp)
    end
    return mut_info
end

export mutual_information_se2
function mutual_information_se2(df::DF)
    mut_info = zeros(df.n, df.n)
    for xi = 1:df.n, yi = 1:df.n, hi = 1:36
        x = (xi - 0.5) * df.cell_size
        y = (yi - 0.5) * df.cell_size
        h = (hi - 0.5) * 10.0
        # TODO: really, should have some other heading
        xp = (x, y, h)
        mut_info[xi,yi] = mutual_information(df, xp)
    end
    return mut_info
end





# TODO: I have no idea if anything below is valid



# this version is valid if the sensor noise is the same regardless
#  of the vehicle and sensor locations
# TODO: is this remotely valid
export mutual_information2
function mutual_information2(df::DF, xp::Pose)
	H_o = 0.0

	for o in df.obs_list
		po = p_obs(df, xp, o)
		if po > 0.0
			H_o -= po * log(po)
		end

	end
	return H_o
end


# for FOV stuff... this is ugly
function mutual_information3(m::SearchDomain, uav::Vehicle, df::DF)
	mut_info = zeros(df.n, df.n, df.n)
	for xv = 1:df.n
		x = (xv - 0.5) * df.cell_size
		for yv = 1:df.n
			y = (yv - 0.5) * df.cell_size
			for zv = 1:df.n
				z = (zv - 0.5) * (360./21.0)
				xp = (x, y, z)
				mut_info[xv,yv,zv] = mutual_information(m, uav, df, xp)
			end
		end
	end
	return mut_info
end

function mutual_information2(m::SearchDomain, uav::Vehicle, df::DF)
	mut_info = zeros(df.n, df.n)
	for xv = 1:df.n
		x = (xv - 0.5) * df.cell_size
		for yv = 1:df.n
			y = (yv - 0.5) * df.cell_size
			# TODO: really, should have some other heading
			xp = (x, y, 0.0)
			mut_info[xv,yv] = mutual_information2(m, uav, df, xp)
		end
	end
	return mut_info
end
