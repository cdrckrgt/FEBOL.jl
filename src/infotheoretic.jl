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
				xj = (theta_x-1) * df.cell_size + df.cell_size/2.0
				yj = (theta_y-1) * df.cell_size + df.cell_size/2.0
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

		# sum over theta
		for theta_x = 1:df.n
			for theta_y = 1:df.n
				xj = (theta_x-1) * df.cell_size + df.cell_size/2.0
				yj = (theta_y-1) * df.cell_size + df.cell_size/2.0

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
# TODO: update for the FEBOL package
function mutual_information(m::SearchDomain)
	mut_info = zeros(m.num_cells, m.num_cells)
	for xv = 1:m.num_cells
		for yv = 1:m.num_cells
			mut_info[xv,yv] = mutual_information(m, xv-1.0, yv-1.0)
		end
	end
	return mut_info
end
