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

# TODO: I think this can be made faster by checking that df.b[xj,yj] > 0
function update!(df::DF, x::Vehicle, o::Float64)
	od = deg2bin(o, df)
	num_cells = df.n
	bp_sum = 0.0

	for theta_x = 1:num_cells
		for theta_y = 1:num_cells
			# convert grid cell number to actual location
			if df.b[theta_x, theta_y] > 0.0
				tx = (theta_x-1) * df.cell_size + df.cell_size/2.0
				ty = (theta_y-1) * df.cell_size + df.cell_size/2.0
				df.b[theta_x, theta_y] *= O(x, (tx, ty), od, df)
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


reset!(f::DF) = fill!(f.b, 1.0/(f.n*f.n))


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
function O(x::Vehicle, theta::LocTuple, o::Obs, df::DF)

	# Calculate true bearing, and find distance to bin edges
	ang_deg = true_bearing(x, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o, df)

	# now look at probability
	d = Normal(0, x.noise_sigma)
	p = cdf(d, rel_end) - cdf(d, rel_start)
	return p
end

# TODO: don't jut assume noise here
function O(xv::Float64, yv::Float64, theta, o::Obs, df::DF)

	# Calculate true bearing, and find distance to bin edges
	xp = (xv, yv)
	ang_deg = true_bearing(xp, theta)
	rel_start, rel_end = rel_bin_edges(ang_deg, o, df)

	# now look at probability
	#p = cdf(m.d, deg2rad(rel_end)) - cdf(m.d, deg2rad(rel_start))
	d = Normal(0, 10.0)
	p = cdf(d, rel_end) - cdf(d, rel_start)
	return p
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
function bin2deg(bin_deg::Int, df::DF)
	full_bin = 360.0 / df.num_bins
	half_bin = full_bin / 2.0
	if bin_deg == 0
		start_val = -half_bin
		end_val = half_bin
	else
		start_val = full_bin * bin_deg - half_bin
		end_val  = full_bin * bin_deg + half_bin
	end
	return start_val, end_val
end

# Find the relative offset
# TODO: must account for different discretizations
function rel_bin_edges(bearing_deg, o::Obs, df::DF)

	# calculate start, end degrees of bin
	start_deg, end_deg = bin2deg(o, df)

	# compute relative distance to true bearing
	rel_start = fit_180(bearing_deg - start_deg)
	rel_end = fit_180(bearing_deg - end_deg)

	#rel_start = min(abs(rel_start), abs(rel_end))
	#rel_end = rel_start + 360.0 / df.num_bins

	# Make sure start is further left on number line
	if rel_end < rel_start
		temp = rel_start
		rel_start = rel_end
		rel_end = temp
	end

	# If we straddle the wrong point
	# Say df.num_bins = 10, and rel_start = -175, rel_end = 175
	# rel_end - rel_start would be 350 degrees, but this should be 10
	# so set rel_start to 175 and rel_end to 185
	# TODO: I'm pretty sure this doesn't work for df.num_bins = 2
	if (rel_end - rel_start) - 1e-3 > (360.0/df.num_bins)
		rel_start = rel_end
		rel_end += 360.0/df.num_bins
	end

	return rel_start, rel_end
end

# Fits an angle into -180 to 180
# Assume the angle is within -360 to 360
function fit_180(angle::Float64)
	if angle > 180.0
		angle = 360.0 - angle
	elseif angle < -180.0
		angle += 360.0
	end
	return angle
end
