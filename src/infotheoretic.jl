######################################################################
# infotheoretic.jl
# has things like mutual information and stuff
######################################################################

export p_obs
# sum over all possible jammer locations
function p_obs(m::SearchDomain, x::Vehicle, df::DF, xp::Pose, o::ObsBin)
	prob = 0.0
	for theta_x = 1:df.n
		for theta_y = 1:df.n
			if df.b[theta_x,theta_y] > 0.0
				xj = (theta_x-0.5) * df.cell_size
				yj = (theta_y-0.5) * df.cell_size
				prob += df.b[theta_x, theta_y] * O(x, x.sensor, xp, (xj,yj), o, df)
			end
		end
	end
	return prob
end

# computes mutual information for a specific vehicle location
# xp is a proposed pose
# really need to loop over all possible observations
export mutual_information
function mutual_information(m::SearchDomain, x::Vehicle, df::DF, xp::Pose)
	H_o = 0.0
	H_o_t = 0.0
	#for o = 0:df.num_bins
	#for o = -20:20  # for DirOmni sensor
	for o in df.bin_range
		po = p_obs(m, x, df, xp, o)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over possible jammer locations
		for theta_x = 1:df.n
			for theta_y = 1:df.n
				xj = (theta_x - 0.5) * df.cell_size
				yj = (theta_y - 0.5) * df.cell_size

				if df.b[theta_x, theta_y] > 0.0
					pot = O(x, x.sensor, xp, (xj,yj), o, df)
					if pot > 0.0
						H_o_t -= pot * df.b[theta_x, theta_y] * log(pot)
					end
				end
			end
		end
	end
	return H_o - H_o_t
end

# computes mutual information for all locations
# the provided discrete filter gives the domain size and belief
# I need a Vehicle or Sensor to pass into the observation function
function mutual_information(m::SearchDomain, uav::Vehicle, df::DF)
	mut_info = zeros(df.n, df.n)
	for xv = 1:df.n
		x = (xv - 0.5) * df.cell_size
		for yv = 1:df.n
			y = (yv - 0.5) * df.cell_size
			xp = (uav.x, uav.y, uav.heading)
			mut_info[xv,yv] = mutual_information(m, uav, df, xp)
		end
	end
	return mut_info
end
