######################################################################
# ekf.jl
# handles kalman filter stuff
######################################################################

# length is size of one side of search domain
type EKF <: AbstractFilter
	mu::Vector{Float64}			# belief
	Sigma::Matrix{Float64}		# belief
	length::Float64	

	function EKF(m::SearchDomain)
		mu = m.length/2 * ones(2)
		S = 1e9 * eye(2)
		return new(mu, S, m.length)
	end
end

# have to include search domain because jammer location factors in
# Really, I should just fold that into the state
# TODO: just subtracting is not ok, need circle distance (can be negative)
function update!(ekf::EKF, x::Vehicle, o::Float64)
	xr = ekf.mu[1] - x.x
	yr = ekf.mu[2] - x.y
	if xr == 0.0 && yr === 0.0
		xr = 1e-6
		yr = 1e-6
	end
	Ht = [yr, -xr]' * (180.0 / pi) / (xr^2 + yr^2)
	Kt = ekf.Sigma * Ht' * inv(Ht * ekf.Sigma * Ht' + 100.)

	o_predict = true_bearing(x, ekf.mu) 
	mu_t = ekf.mu + Kt * (o - o_predict)
	Sigma_t = (eye(2)  - Kt * Ht) * ekf.Sigma

	# TODO: use copy here
	ekf.mu = vec(mu_t)
	ekf.Sigma = Sigma_t
end
