######################################################################
# discrete.jl
# Handles stuff for discrete filters
######################################################################

# n is the number of cells per side
# cell_size is size of a single cell
type DF
	n::Int64
	cell_size::Float64

	DF(m::SearchDomain, n::Int64) = new(n, m.length/n)
end
initial_belief(df::DF) = ones(df.n, df.n) / (df.n * df.n)

# This is really just a Matrix....
typealias DiscreteBelief Matrix{Float64}

# do we need search domain to compare size to theta integer
function update!(b::DiscreteBelief, df::DF, x::Vehicle, o::Obs)
	num_cells = df.n
	bp_sum = 0.0

	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			# convert grid cell number to actual location
			tx = (theta_x - 1) * df.cell_size + df.cell_size/2.0
			ty = (theta_y - 1) * df.cell_size + df.cell_size/2.0
			b[theta_x, theta_y] *= O(x, (tx, ty), o)
			bp_sum += b[theta_x, theta_y]
		end
	end

	# normalize
	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			b[theta_x, theta_y] /= bp_sum
		end
	end
end
