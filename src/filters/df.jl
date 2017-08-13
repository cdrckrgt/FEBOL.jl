######################################################################
# df.jl
#
# Discrete filter
######################################################################

# n is the number of cells per side
# cell_size is size of a single cell
type DF <: AbstractFilter
	b::Matrix{Float64}
	n::Int64
	cell_size::Float64
	num_bins::Int64
	bin_range::UnitRange{Int64}
	sensor::Sensor

	function DF(m::SearchDomain, n::Int64, num_bins::Int64=36)
		b = ones(n, n) / (n * n)
		return new(b, n, m.length/n, num_bins, 0:(num_bins-1), BearingOnly(10))
	end
	function DF(m::SearchDomain, n::Int64, bin_range::UnitRange{Int64})
		b = ones(n, n) / (n * n)
		num_bins = length(bin_range)
		return new(b, n, m.length/n, num_bins, bin_range, BearingOnly(10))
	end

	function DF(m::SearchDomain, n::Int64, sensor::Sensor, num_bins::Int64=36)
		b = ones(n, n) / (n * n)
		return new(b, n, m.length/n, num_bins, 0:(num_bins-1), sensor)
	end
	function DF(m::SearchDomain, n::Int64, sensor::Sensor, bin_range::UnitRange{Int64})
		b = ones(n, n) / (n * n)
		num_bins = length(bin_range)
		return new(b, n, m.length/n, num_bins, bin_range, sensor)
	end
end

# TODO: I think this can be made faster by checking that df.b[xj,yj] > 0
function update!(df::DF, x::Vehicle, o::Float64)
	ob = obs2bin(o, df.sensor)
	num_cells = df.n
	bp_sum = 0.0

	p = (x.x, x.y, x.heading)

	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			# convert grid cell number to actual location
			if df.b[theta_x, theta_y] > 0.0
				tx = (theta_x-0.5) * df.cell_size
				ty = (theta_y-0.5) * df.cell_size
				df.b[theta_x, theta_y] *= O(df.sensor, (tx,ty), p, ob)
				bp_sum += df.b[theta_x, theta_y]
			end
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
# Assumes that the belief sums to one
function centroid(df::DF)
	x_val = 0.0; y_val = 0.0
	for x = 1:df.n
		for y = 1:df.n
			x_val += (x-.5) * df.b[x,y]
			y_val += (y-.5) * df.b[x,y]
		end
	end
	return x_val*df.cell_size, y_val*df.cell_size
end

function centroid(d::Matrix{Float64}, L::Float64)
	x_val = 0.0; y_val = 0.0
	n = size(d,1)
	cell_size = L / n
	d_sum = 0.0
	for x = 1:n
		for y = 1:n
			x_val += (x-.5) * d[x,y]
			y_val += (y-.5) * d[x,y]
			d_sum += d[x,y]
		end
	end
	return x_val*cell_size/d_sum, y_val*cell_size/d_sum
end

function covariance(df::DF)
	mu_x = mu_y = 0.0
	c_xx = c_xy = c_yy = 0.0
	for xi = 1:df.n
		for yi = 1:df.n
			x = (xi-0.5)*df.cell_size
			y = (yi-0.5)*df.cell_size

			mu_x += x * df.b[xi,yi]
			mu_y += y * df.b[xi,yi]

			c_xx += df.b[xi,yi] * x * x
			c_yy += df.b[xi,yi] * y * y
			c_xy += df.b[xi,yi] * x * y
		end
	end
	c_xx -= (mu_x * mu_x)
	c_yy -= (mu_y * mu_y)
	c_xy -= (mu_x * mu_y)
	return [c_xx+1e-4 c_xy; c_xy c_yy+1e-4]
end


# add some to ensure we are 
function covariance(d::Matrix{Float64}, L::Float64)
	mu_x, mu_y = centroid(d, L)
	c_xx = c_xy = c_yy = 0.0
	n = size(d,1)
	cell_size = L / n
	d_sum = 0.0
	for xi = 1:n
		for yi = 1:n
			x = (xi-0.5)*cell_size
			y = (yi-0.5)*cell_size

			c_xx += d[xi,yi] * x * x
			c_yy += d[xi,yi] * y * y
			c_xy += d[xi,yi] * (x - mu_x) * (y - mu_y)
			d_sum += d[xi,yi]
		end
	end
	c_xx = c_xx/d_sum - (mu_x * mu_x)
	c_yy = c_yy/d_sum - (mu_y * mu_y)
	c_xy = c_xy/d_sum

	# add 1e-3 to ensure we are positive definite
	return [c_xx+1e-3 c_xy; c_xy c_yy+1e-3]
end


# Returns the entropy of the distribution.
# Could just borrow this from Distributions.jl
function entropy(df::DF)
	ent = 0.0
	for xj = 1:df.n
		for yj = 1:df.n
			prob = df.b[xj,yj]
			if prob > 0.0
				ent -= prob*log(prob)
			end
		end
	end
	return ent / log(df.n * df.n)
end


reset!(f::DF) = fill!(f.b, 1.0/(f.n*f.n))

function print_belief(df::DF)
	for y = df.n:-1:1
		for x = 1:(df.n-1)
			@printf "%.2f," df.b[x,y]
		end
		@printf "%.2f\n" df.b[df.n,y]
	end
	#@printf "%.2f", 
end


# I never use these...
function noiseless(x::Vehicle, theta::LocTuple)
	noiseless(x, x.sensor, theta)
end
function noiseless(x::Vehicle, s::BearingOnly, theta::LocTuple)
	true_bearing(x, theta)
end
function noiseless(x::Vehicle, s::DirOmni, theta::LocTuple)
	rel_bearing = x.heading - true_bearing(x, theta)
	if rel_bearing < 0.0
		rel_bearing += 360.0
	end
	rel_int = round(Int, rel_bearing, RoundDown) + 1

	return s.means[rel_int]
end
