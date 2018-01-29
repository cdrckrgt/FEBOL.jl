######################################################################
# infotheoretic.jl
# has things like mutual information and stuff
######################################################################

export mutual_information3
export p_obs
# sum over all possible jammer locations
function p_obs(df::DF, xp::Pose, o::ObsBin)
	prob = 0.0
	for txi = 1:df.n, tyi = 1:df.n
        if df.b[txi,tyi] > 0.0
            tx = (txi-0.5) * df.cell_size
            ty = (tyi-0.5) * df.cell_size
            prob += df.b[txi, tyi] * O(df.sensor, (tx,ty), xp, o)
        end
	end
	return prob
end

# computes mutual information for a specific vehicle location
# xp is a proposed pose
# really need to loop over all possible observations
export mutual_information
function mutual_information(df::DF, xp::Pose)
	H_o = 0.0
	H_o_t = 0.0

	for o in df.bin_range
		po = p_obs(df, xp, o)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over possible jammer locations
		for txi = 1:df.n, tyi = 1:df.n
            tx = (txi - 0.5) * df.cell_size
            ty = (tyi - 0.5) * df.cell_size

            if df.b[txi, tyi] > 0.0
                pot = O(df.sensor, (tx,ty), xp, o)
                if pot > 0.0
                    H_o_t -= pot * df.b[txi, tyi] * log(pot)
                end
            end
		end
	end
	return H_o - H_o_t
end


# computes mutual information for all locations
# the provided discrete filter gives the domain size and belief
# I need a Vehicle or Sensor to pass into the observation function
function mutual_information(df::DF)
    mut_info = zeros(df.n, df.n)
    for xv = 1:df.n, yv = 1:df.n
        x = (xv - 0.5) * df.cell_size
        y = (yv - 0.5) * df.cell_size
        # TODO: really, should have some other heading
        xp = (x, y, 0.0)
        mut_info[xv,yv] = mutual_information(df, xp)
    end
    return mut_info
end

# this version is valid if the sensor noise is the same regardless
#  of the vehicle and sensor locations
export mutual_information2
function mutual_information2(m::SearchDomain, x::Vehicle, df::DF, xp::Pose)
	H_o = 0.0

	for o in x.sensor.bin_range
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


# fisher information
export fisher_det

# TODO: mega wrong, don't pass in the uav's sensor, pass in the filter's
fisher_det(uav::Vehicle, df::DF) = fisher_det(uav.sensor, df)

function fisher_det(bo::BearingOnly, df::DF)
	fd = zeros(df.n, df.n)
	ns2 = bo.noise_sigma * bo.noise_sigma
	for xind = 1:df.n
		x = (xind - 0.5) * df.cell_size
		for yind = 1:df.n
			y = (yind - 0.5) * df.cell_size

			# now we must loop over belief
			a = bc = d = 0.0
			for txi = 1:df.n
				tx = (txi - 0.5) * df.cell_size
				for tyi = 1:df.n
					ty = (tyi - 0.5) * df.cell_size
					p = df.b[txi,tyi]
					dx = tx - x
					dy = ty - y
					if dx == 0.0
						dx = 1e-6
					end
					if dy == 0.0
						dy = 1e-6
					end
					r2 = dx*dx + dy*dy
					r4 = r2*r2
					
					a += p*dy*dy / r4
					bc -= p*dx*dy / r4
					d += p*dx*dx / r4
				end
			end
			a /= ns2
			bc /= ns2
			d /= ns2

			fd[xind, yind] = a*d - bc*bc

		end
	end
	return fd
end


function fisher_det(bo::RangeOnly, df::DF)
	fd = zeros(df.n, df.n)
	ns2 = bo.noise_sigma * bo.noise_sigma
	for xind = 1:df.n
		x = (xind - 0.5) * df.cell_size
		for yind = 1:df.n
			y = (yind - 0.5) * df.cell_size

			# now we must loop over belief
			a = bc = d = 0.0
			for txi = 1:df.n
				tx = (txi - 0.5) * df.cell_size
				for tyi = 1:df.n
					ty = (tyi - 0.5) * df.cell_size
					p = df.b[txi,tyi]
					dx = tx - x
					dy = ty - y
					if dx == 0.0
						dx = 1e-6
					end
					if dy == 0.0
						dy = 1e-6
					end
					r2 = dx*dx + dy*dy
					
					a += p*dx*dx / r2
					bc += p*dx*dy / r2
					d += p*dy*dy / r2
				end
			end
			a /= ns2
			bc /= ns2
			d /= ns2

			fd[xind, yind] = a*d - bc*bc

		end
	end
	return fd
end
