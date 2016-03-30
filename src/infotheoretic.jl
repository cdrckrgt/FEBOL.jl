######################################################################
# infotheoretic.jl
# has things like mutual information and stuff
######################################################################

# sum over all possible jammer locations
function p_obs(m::SearchDomain, df::DF, xv::Float64, yv::Float64, o::Obs)
	prob = 0.0
	for xj = 1:df.n
		for yj = 1:df.n
			#prob += df.b[xj,yj] * O(m, xv, yv, (xj-1,yj-1), o)
			prob += df.b[xj,yj] * O(m, xv, yv, (xj-.5,yj-.5), o)
		end
	end
	return prob
end

# computes mutual information for a specific vehicle location
function mutual_information(m::SearchDomain, df::DF, xv::Float64, yv::Float64)
	H_o = 0.0
	H_o_t = 0.0
	for o = 0:35
		po = p_obs(m, df, xv, yv, o)
		if po > 0.0
			H_o -= po * log(po)
		end

		# sum over theta
		for xj = 1:df.n
			for yj = 1:df.n
				#pot = O(m, xv, yv, (xj-1,yj-1), o)
				pot = O(m, xv, yv, (xj-.5,yj-.5), o)
				if pot > 0.0
					H_o_t -= pot * df.b[xj,yj] * log(pot)
				end
			end
		end
	end
	return H_o - H_o_t
end

# computes mutual information for all locations
function mutual_information(m::SearchDomain)
	mut_info = zeros(m.num_cells, m.num_cells)
	for xv = 1:m.num_cells
		for yv = 1:m.num_cells
			mut_info[xv,yv] = mutual_information(m, xv-1.0, yv-1.0)
		end
	end
	return mut_info
end
