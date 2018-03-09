######################################################################
# df.jl
#
# Discrete filter
######################################################################

struct DF{OL, S<:Sensor} <: AbstractFilter
    b::Matrix{Float64}      # the actual discrete belief
    n::Int64                # number of cells per side
    cell_size::Float64      # width of each cell, in meters
    sensor::S               # sensor model used in filtering
    obs_list::OL            # list of observations
end
function DF(m::SearchDomain, n::Int64, sensor::Sensor, obs_list=0:0)
    b = ones(n, n) / (n * n)
    return DF(b, n, m.length/n, sensor, obs_list)
end


function update!(df::DF, x::Vehicle, o)
    n = df.n
    bp_sum = 0.0

    p = (x.x, x.y, x.heading)

    for txi = 1:n, tyi = 1:n
        if df.b[txi, tyi] > 0.0
            tx = (txi-0.5) * df.cell_size
            ty = (tyi-0.5) * df.cell_size
            df.b[txi, tyi] *= O(df.sensor, (tx,ty,0.0,0.0), p, o)
            bp_sum += df.b[txi, tyi]
        end
    end

    # normalize
    for txi = 1:n, tyi = 1:n
        df.b[txi, tyi] /= bp_sum
    end
end


# returns x, y value
# Assumes that the belief sums to one
function centroid(df::DF)
    x_val = 0.0; y_val = 0.0
    for x = 1:df.n, y = 1:df.n
        x_val += (x-.5) * df.b[x,y]
        y_val += (y-.5) * df.b[x,y]
    end
    return x_val*df.cell_size, y_val*df.cell_size
end

function centroid(d::Matrix{Float64}, L::Float64)
    x_val = 0.0; y_val = 0.0
    n = size(d,1)
    cell_size = L / n
    d_sum = 0.0
    for x = 1:n, y = 1:n
        x_val += (x-.5) * d[x,y]
        y_val += (y-.5) * d[x,y]
        d_sum += d[x,y]
    end
    return x_val*cell_size/d_sum, y_val*cell_size/d_sum
end

function covariance(df::DF)
    mu_x = mu_y = 0.0
    c_xx = c_xy = c_yy = 0.0
    for xi = 1:df.n, yi = 1:df.n
        x = (xi-0.5)*df.cell_size
        y = (yi-0.5)*df.cell_size

        mu_x += x * df.b[xi,yi]
        mu_y += y * df.b[xi,yi]

        c_xx += df.b[xi,yi] * x * x
        c_yy += df.b[xi,yi] * y * y
        c_xy += df.b[xi,yi] * x * y
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
entropy(df::DF) = entropy(df.b)

# I had this before... a normalized version?
#entropy(df::DF) = entropy(df.b) / log(df.n * df.n)


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
