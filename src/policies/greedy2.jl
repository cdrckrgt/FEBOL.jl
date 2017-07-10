######################################################################
# greedy2.jl
# 
# considers bearing only where we have to rotate
# Only assume bearing only sensor at this time
#
# Defines a greedy, info-theoretic policy
# Currently only considers a small set of the possible actions
######################################################################
# eval_type = 1 (discrete greedy maximization)
# eval_type = 2 (determinant)
# eval_type = 3 (largest eigenvalue)
type GreedyPolicy2 <: Policy
	eval_type::Int
end


# loop over all actions.
# The one with smallest expected entropy is best
# We know this depends on sensor
function action(m::SearchDomain, v::Vehicle, o::Float64, f::DF, p::GreedyPolicy2)
	best_mi = -Inf
	best_x = v.x
	best_y = v.y

	mu = centroid(f)
	mu_x = mu[1]
	mu_y = mu[2]
	Omega = inv(covariance(f))
	a1 = Omega[1,1]
	b1 = Omega[1,2]
	c1 = Omega[2,1]
	d1 = Omega[2,2]
	mi = 0.

	for xi = 1:f.n
		x = (xi - 0.5) * f.cell_size
		for yi = 1:f.n
			y = (yi - 0.5) * f.cell_size

			if p.eval_type == 1
				xp = (x,y,0.0)
				mi = mutual_information2(m, v, f, xp)
			else
				xr = mu_x - x
				yr = mu_y - y

				q = 180.0 / (pi * (xr*xr + yr*yr))
				q = q*q / v.sensor.noise_sigma;

				sum_a = a1 + q*(yr*yr)
				sum_b = b1 - q*(xr*yr)
				sum_c = c1 - q*(xr*yr)
				sum_d = d1 + q*(xr*xr)
				if p.eval_type == 2
					mi = sum_a*sum_d - sum_b*sum_c
				else
					mi = smallest_eig(sum_a, sum_b, sum_c, sum_d)
				end
			end

			#println("mi = ", mi)
			if mi > best_mi
				best_mi = mi
				best_x = x
				best_y = y
			end
		end
	end
	return (best_x-v.x, best_y-v.y, 0.0)
end

function smallest_eig(a::Float64, b::Float64, c::Float64, d::Float64)
	B = -1.0 * (d+a)
	C = a*d - b*c
	a1 = (-B + sqrt(B*B - 4C)) / 2.0
	a2 = (-B - sqrt(B*B - 4C)) / 2.0

	if a1 < a2
		return a1
	end
	return a2
end
