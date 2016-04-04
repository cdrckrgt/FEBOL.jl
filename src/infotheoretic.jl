######################################################################
# infotheoretic.jl
# has things like mutual information and stuff
######################################################################

# sum over all possible jammer locations
function p_obs(m::SearchDomain, df::DF, xv::Float64, yv::Float64, o::Obs)
	prob = 0.0
	for theta_x = 1:df.n
		for theta_y = 1:df.n
			if df.b[theta_x,theta_y] > 0.0
				xj = (theta_x - 1) * df.cell_size + df.cell_size/2.0
				yj = (theta_y - 1) * df.cell_size + df.cell_size/2.0
				prob += df.b[theta_x, theta_y] * O(xv, yv, (xj, yj), o, df)
			end
		end
	end
	return prob
end

# computes mutual information for a specific vehicle location
# TODO: handle different discretization levels
function mutual_information(m::SearchDomain, df::DF, xv::Float64, yv::Float64)
	H_o = 0.0
	H_o_t = 0.0
	for o = 0:df.num_bins
		po = p_obs(m, df, xv, yv, o)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over theta
		for theta_x = 1:df.n
			for theta_y = 1:df.n
				xj = (theta_x - 1) * df.cell_size + df.cell_size/2.0
				yj = (theta_y - 1) * df.cell_size + df.cell_size/2.0

				if df.b[theta_x, theta_y] > 0.0
					pot = O(xv, yv, (xj, yj), o, df)
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
