######################################################################
# ukf.jl
# All about that unscented Kalman filter
######################################################################
#include("kalman.jl")

type UKF <: GaussianFilter
	mu::Vector{Float64}			# belief
	Sigma::Matrix{Float64}		# belief
	length::Float64	
	lambda::Float64
	n::Int

	w0::Float64
	wi::Float64
	initialized::Bool
	initializer::Initializer

	#function UKF(m::SearchDomain, alpha::Float64, k::Float64)
	function UKF(m::SearchDomain)
		mu = m.length/2 * ones(2)
		S = 1e5 * eye(2)

		n = 2
		lambda = 2.

		# create w
		w0 = lambda / (n + lambda)
		wi = 0.5 / (n + lambda)

		initer = NaiveInitializer(m)
		return new(mu, S, m.length, lambda, n, w0, wi,false,initer)
	end
end


function update!(ukf::UKF, x::Vehicle, o::Float64)

	# check if initialized..
	if !ukf.initialized
		check_initialization(ukf, x, o)
		return nothing
	end

	# for ease of writing
	w0 = ukf.w0
	wi = ukf.wi

	# Prediction step
	# nothing changes, because jammer doesn't go anywhere

	# Update step
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

	# TODO: check if the imaginary parts are too big
	Z0 = true_bearing(x, real(X0))
	Z1 = true_bearing(x, real(X1))
	Z2 = true_bearing(x, real(X2))
	Z3 = true_bearing(x, real(X3))
	Z4 = true_bearing(x, real(X4))

	# Compute z_hat and differences
	# TODO: I'm not sure this is valid for circular domain
	#zhat = w0*Z0 + wi*(Z1 + Z2 + Z3 + Z4)
	zhat = angle_mean((Z0,Z1,Z2,Z3,Z4), (w0,wi,wi,wi,wi))

	Z0_diff = fit_180(Z0 - zhat)
	Z1_diff = fit_180(Z1 - zhat)
	Z2_diff = fit_180(Z2 - zhat)
	Z3_diff = fit_180(Z3 - zhat)
	Z4_diff = fit_180(Z4 - zhat)

	# Compute St
	# TODO: I'm not sure this is valid for circular domain
	Qt = x.sensor.noise_sigma * x.sensor.noise_sigma
	St = w0*Z0_diff^2 + wi*(Z1_diff^2 + Z2_diff^2 + Z3_diff^2 + Z4_diff^2) 
	St += Qt

	# Compute Sigma_xz
	# TODO: Not valid for circular domain?
	Sigma_xz = wi * (sqrtSigma1) * Z1_diff
	Sigma_xz += wi * (sqrtSigma2) * Z2_diff
	Sigma_xz -= wi * (sqrtSigma1) * Z3_diff
	Sigma_xz -= wi * (sqrtSigma2) * Z4_diff

	# Compute gain, mean, and standard deviation
	Kt = Sigma_xz * inv(St)

	ukf.mu = mu + Kt*fit_180(o - zhat)
	#ukf.Sigma = Sigma_bar - Kt*St*Kt'
	# below is same as above, but simpler
	ukf.Sigma = ukf.Sigma - Sigma_xz * inv(St) * Sigma_xz'

	#println("ukf.mu = ", round(ukf.mu,2))
	#println("ukf.Sigma = ", round(ukf.Sigma,2))
	return nothing
end
