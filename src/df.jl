######################################################################
# df.jl
# Discrete filters
######################################################################

# n is the number of cells per side
# cell_size is size of a single cell
type DF <: AbstractFilter
	b::Matrix{Float64}
	n::Int64
	cell_size::Float64

	function DF(m::SearchDomain, n::Int64)
		b = ones(n, n) / (n * n)
		return new(b, n, m.length/n)
	end
end
initial_belief(df::DF) = ones(df.n, df.n) / (df.n * df.n)

# This is really just a Matrix....
typealias DiscreteBelief Matrix{Float64}

# do we need search domain to compare size to theta integer
function update!(df::DF, x::Vehicle, o::Obs)
	num_cells = df.n
	bp_sum = 0.0

	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			# convert grid cell number to actual location
			tx = (theta_x - 1) * df.cell_size + df.cell_size/2.0
			ty = (theta_y - 1) * df.cell_size + df.cell_size/2.0
			df.b[theta_x, theta_y] *= O(x, (tx, ty), o)
			bp_sum += df.b[theta_x, theta_y]
		end
	end

	# normalize
	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			df.b[theta_x, theta_y] /= bp_sum
		end
	end
end

# returns x, y value
function centroid(df::DF)
	x_val = 0.0; y_val = 0.0
	x_sum = 0.0; y_sum = 0.0
	for x = 1:df.n
		for y = 1:df.n
			x_val += (x-.5) * df.b[x,y]
			x_sum += df.b[x,y]
			y_val += (y-.5) * df.b[x,y]
			y_sum += df.b[x,y]
		end
	end
	return x_val*df.cell_size / x_sum, y_val*df.cell_size / y_sum
end