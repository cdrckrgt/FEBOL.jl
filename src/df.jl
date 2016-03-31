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

# This is really just a Matrix....
typealias DiscreteBelief Matrix{Float64}

# do we need search domain to compare size to theta integer
# TODO: sanitize the obervation inside
function update!(df::DF, x::Vehicle, o::Float64)
	od = discretize_obs(o, df)
	num_cells = df.n
	bp_sum = 0.0

	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			# convert grid cell number to actual location
			tx = (theta_x - 1) * df.cell_size + df.cell_size/2.0
			ty = (theta_y - 1) * df.cell_size + df.cell_size/2.0
			df.b[theta_x, theta_y] *= O(x, (tx, ty), od)
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

# TODO: use level of discretization provided by df
# Assume observation is within [0,360)
# 355-5 = 0
# 5 -15 = 1
# 15-25 = 2
function discretize_obs(o::Float64, df::DF)
	od = round(Int, o/10.0, RoundNearestTiesUp)
	if od == 36
		od = 0
	end
	return od
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

# Returns the entropy of the distribution.
function entropy(df::DF)
end
