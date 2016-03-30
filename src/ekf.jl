######################################################################
# ekf.jl
# handles kalman filter stuff
######################################################################

type Gaussian <: Belief
	mean::Vector{Float64}
	Sigma::Matrix{Float64}
end

type EKF
end

function initial_belief(kf::EKF)
	return Gaussian([5.,5.], [100000. 0; 0 100000.0])
end

# have to include search domain because jammer location factors in
# Really, I should just fold that into the state
# TODO: just subtracting is not ok, need circle distance (can be negative)
function update!(m::SearchDomain,b::Gaussian, df::EKF, x::Vehicle, o::Obs)
	xr = m.theta[1] - x.x
	yr = m.theta[2] - x.y
	Ht = [yr, -xr]' * (180.0 / pi) / (xr^2 + yr^2)
	Kt = b.Sigma * Ht' * inv(Ht * b.Sigma * Ht' + 100.)

	mu_t = b.mean + Kt * (o - true_bearing((x.x,x.y), m.theta) )
	Sigma_t = (eye(2)  - Kt * Ht) * b.Sigma

	b.mean = vec(mu_t)
	b.Sigma = Sigma_t
end
