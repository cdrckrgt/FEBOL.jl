######################################################################
# gaussian_mpc.jl
######################################################################

type GaussianMPC <: Policy
	n::Int   # what it meant before
	N::Int   # planning horizon
	actions::Vector{Action}

	GaussianMPC(x::Vehicle) = GaussianMPC(x, 8)
	GaussianMPC(x::Vehicle, n::Int) = GaussianMPC(x, 8, 4)

	# create list of actions
	function GaussianMPC(x::Vehicle, n::Int, N::Int)
		angles = linspace(0.0, 360 - 360/n, n)
		actions = Array(Action, n)
		for i = 1:n
			ax = x.max_step * sind(angles[i])
			ay = x.max_step * cosd(angles[i])
			actions[i] = (ax, ay, 0.0)
		end

		return new(n, N, actions)
	end

end

function action(m::SearchDomain, x::Vehicle, o::Float64, f::AbstractFilter, p::GaussianMPC)

	# create Gaussian approximation of belief
	mu = centroid(f)
	Sigma = covariance(f)

	# convert Sigma to Omega and calculate Rinv
	Omega = inv(Sigma)
	Rinv = inv(x.sensor.noise_sigma)
	xp = (x.x, x.y, x.heading)

	# do a tree search here..
	max_det = 0.0
	best_action = p.actions[1]
	for a in p.actions
		temp_det = rec_det(m,xp,a,p.N,Omega,Rinv,mu,p)
		if temp_det > max_det
			max_det = temp_det
			best_action = a
		end
	end

	return best_action
end

# Recursive determinant function
# xp = pose of vehicle
# a  = action
# N  = planning horizon
# Omega = information matrix
function rec_det(m::SearchDomain, xp::Pose, a::Action, N::Int, Omega::Matrix{Float64}, Rinv::Float64, mu::LocTuple, p::GaussianMPC)
	np = new_pose(m, xp, a)
	Ct = Jacobian(mu, np)
	if N == 0
		return det( Omega + Rinv*(Ct'*Ct) )
	end

	max_det = 0.0
	for a in p.actions

		max_det = max( max_det, rec_det(m, xp, a, N-1, Omega+Rinv*(Ct'*Ct), Rinv, mu, p) )
	end

	return max_det
end

function Jacobian(mu::LocTuple, np::Pose)
	xr = mu[1] - np[1]
	yr = mu[2] - np[2]
	if xr == 0.0 && yr === 0.0
		xr = 1e-6
		yr = 1e-6
	end
	Ct = [yr, -xr]' * (180.0 / pi) / (xr^2 + yr^2)
	return Ct
end
