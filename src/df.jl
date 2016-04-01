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
	num_bins::Int64

	function DF(m::SearchDomain, n::Int64)
		b = ones(n, n) / (n * n)
		return new(b, n, m.length/n, 36)
	end
end

function update!(df::DF, x::Vehicle, o::Float64)
	od = deg2bin(o, df)
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


######################################################################
# Functions required for O()
######################################################################

"""
`O(m::SearchDomain, x::Vehicle, theta, o::Obs)`

Arguments:

 * `m` is a `SearchDomain`
 * `x` is a `Vehicle`
 * `theta` is a possible jammer location
 * `o` is an observation, 0 to 35

Returns probability of observing `o` from `(xp, theta)` in domain `m`.
"""
function O(x::Vehicle, theta::LocTuple, o::Obs)

	# Calculate true bearing, and find distance to bin edges
	ang_deg = true_bearing(x, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o)

	# now look at probability
	d = Normal(0, x.noise_sigma)
	p = cdf(d, rel_end) - cdf(d, rel_start)
	return p
end


# TODO: use level of discretization provided by df
# Assume observation is within [0,360)
# 355-5 = 0
# 5 -15 = 1
# 15-25 = 2
function old_discretize_obs(o::Float64, df::DF)
	od = round(Int, o/10.0, RoundNearestTiesUp)
	if od == 36
		od = 0
	end
	return od
end

# 355 - 4.9999 = 0
# 5 - 14.9999 = 1
# 15 - 24.999 = 2
function deg2bin(o::Float64, df::DF)
	full_bin = 360.0 / df.num_bins
	half_bin = full_bin / 2.0

	od = round( Int, div((o + half_bin), full_bin) )
	if od == df.num_bins
		od = 0
	end
	return od
end

# returns (start_deg, end_deg) integer tuple
# TODO: Modify for variable discretization
function bin2deg(bin_deg::Int64)
	if bin_deg == 0
		start_val = -5
		end_val = 5
	else
		start_val = 10bin_deg - 5
		end_val  = 10bin_deg + 5
	end
	return (start_val, end_val)
end

# Find the relative offset
# TODO: must account for different discretizations
function rel_bin_edges(bearing_deg, o::Obs)

	# calculate start, end degrees of bin
	start_deg, end_deg = bin2deg(o)

	# compute relative distance to true bearing
	rel_start = fit_180(bearing_deg - start_deg)
	rel_end = fit_180(bearing_deg - end_deg)

	# Make sure start is further left on number line
	if rel_end < rel_start
		temp = rel_start
		rel_start = rel_end
		rel_end = temp
	end

	# Handle odd case where we stradle 180 degrees off
	if (rel_start < 0) && (rel_end > 100)
		rel_start = rel_end
		rel_end += 10
	end

	return rel_start, rel_end
end
