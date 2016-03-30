######################################################################
# ekf.jl
# handles kalman filter stuff
######################################################################

#type Gaussian <: Belief
#	mean::Vector{Float64}
#	Sigma::Matrix{Float64}
#end

# length is size of one side of search domain
type EKF <: AbstractFilter

	# belief
	mu::Vector{Float64}
	Sigma::Matrix{Float64}

	# extras
	theta::NTuple{2,Float64}
	length::Float64	

	function EKF(m::SearchDomain)
		mu = m.length/2 * ones(2)
		S = 1e9 * eye(2)
		return new(mu, S, m.theta, m.length)
	end
end

function initial_belief(kf::EKF)
	return Gaussian([5.,5.], [100000. 0; 0 100000.0])
end

# have to include search domain because jammer location factors in
# Really, I should just fold that into the state
# TODO: just subtracting is not ok, need circle distance (can be negative)
# I'm not sure this is correct....
function update!(ekf::EKF, x::Vehicle, o::Obs)
	xr = ekf.theta[1] - x.x
	yr = ekf.theta[2] - x.y
	Ht = [yr, -xr]' * (180.0 / pi) / (xr^2 + yr^2)
	Kt = ekf.Sigma * Ht' * inv(Ht * ekf.Sigma * Ht' + 100.)

	mu_t = ekf.mu + Kt * (o - true_bearing((x.x,x.y), ekf.theta) )
	Sigma_t = (eye(2)  - Kt * Ht) * ekf.Sigma

	ekf.mu = vec(mu_t)
	ekf.Sigma = Sigma_t
end
