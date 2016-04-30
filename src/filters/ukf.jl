######################################################################
# ukf.jl
# All about that unscented Kalman filter
######################################################################

type UKF <: AbstractFilter
	mu::Vector{Float64}			# belief
	Sigma::Matrix{Float64}		# belief
	length::Float64	
	lambda::Float64
	n::Int
	wm0::Float64
	wc0::Float64
	wmci::Float64

	function UKF(m::SearchDomain, alpha::Float64, k::Float64)
		mu = m.length/2 * ones(2)
		S = 1e9 * eye(2)

		n = 2
		Beta = 2.0
		lambda = alpha * alpha * (n + k) - n

		# create w
		wm0 = lambda / (lambda + n)
		wc0 = wm0 + (1.0 - alpha*alpha + Beta)
		wmci = 0.5 / (n + lambda)

		return new(mu, S, m.length, lambda, n, wm0, wc0, wmci)
	end
end


function update!(ukf::UKF, x::Vehicle, o::Float64)
	# for ease of writing
	wc0 = ukf.wc0
	wm0 = ukf.wm0
	wmci = ukf.wmci


	# Compute X_bar
	mu = ukf.mu
	sqrtSigma = sqrtm((ukf.n+ukf.lambda) * ukf.Sigma)
	sqrtSigma1 = sqrtSigma[:,1]
	sqrtSigma2 = sqrtSigma[:,2]
	X0 = mu
	X1 = mu + sqrtSigma1
	X2 = mu + sqrtSigma2
	X3 = mu - sqrtSigma1
	X4 = mu - sqrtSigma2

	# Compute mu_bar_t
	mu_bar = wm0*X0 + wmci*(X1 + X2 + X3 + X4)

	# Compute  Sigma_bar
	Sigma_bar = wc0 * (X0 - mu_bar) * (X0 - mu_bar)'
	Sigma_bar += wmci * (X1 - mu_bar) * (X1 - mu_bar)'
	Sigma_bar += wmci * (X2 - mu_bar) * (X2 - mu_bar)'
	Sigma_bar += wmci * (X3 - mu_bar) * (X3 - mu_bar)'
	Sigma_bar += wmci * (X4 - mu_bar) * (X4 - mu_bar)'

	# Compute the sqrtSigmas
	sqrtSigma = sqrtm((ukf.n+ukf.lambda) * Sigma_bar)
	sqrtSigma1 = sqrtSigma[:,1]
	sqrtSigma2 = sqrtSigma[:,2]

	# Compute the Z_bar
	println("Sigma1 = ", sqrtSigma1)
	println("Sigma2 = ", sqrtSigma2)
	println("mu_bar = ", mu_bar)
	println("************************")
	sqrtSigma1 = real(sqrtSigma1)
	sqrtSigma2 = real(sqrtSigma2)
	mu_bar = real(mu_bar)
	Z0 = true_bearing(x, mu_bar)
	Z1 = true_bearing(x, mu_bar + sqrtSigma1)
	Z2 = true_bearing(x, mu_bar + sqrtSigma2)
	Z3 = true_bearing(x, mu_bar - sqrtSigma1)
	Z4 = true_bearing(x, mu_bar - sqrtSigma2)

	# Compute z_hat and differences
	# TODO: I'm not sure this is valid for circular domain
	zhat = wm0*Z0 + wmci*(Z1 + Z2 + Z3 + Z4)
	Z0_diff = fit_180(Z0 - zhat)
	Z1_diff = fit_180(Z1 - zhat)
	Z2_diff = fit_180(Z2 - zhat)
	Z3_diff = fit_180(Z3 - zhat)
	Z4_diff = fit_180(Z4 - zhat)

	# Compute St
	# TODO: I'm not sure this is valid for circular domain
	Qt = x.sensor.noise_sigma * x.sensor.noise_sigma
	St = wc0 * Z0_diff * Z0_diff
	St += wmci * Z1_diff * Z1_diff
	St += wmci * Z2_diff * Z2_diff
	St += wmci * Z3_diff * Z3_diff
	St += wmci * Z4_diff * Z4_diff
	St += Qt

	# Compute Sigma_xz
	# TODO: Not valid for circular domain?
	Sigma_xz = wmci * (sqrtSigma1) * Z1_diff
	Sigma_xz += wmci * (sqrtSigma2) * Z2_diff
	Sigma_xz += wmci * (sqrtSigma1) * Z3_diff
	Sigma_xz += wmci * (sqrtSigma2) * Z4_diff

	# Compute gain, mean, and standard deviation
	Kt = Sigma_xz * inv(St)
	ukf.mu = mu_bar + Kt*fit_180(o - zhat)
	ukf.Sigma = Sigma_bar - Kt*St*Kt'
end
