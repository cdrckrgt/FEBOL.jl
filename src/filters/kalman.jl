######################################################################
# kalman.jl
# helper functions for ekf, ukf, etc
# Each initializer should have:
#  length
#  mu
#  Sigma
######################################################################

abstract GaussianFilter <: AbstractFilter
abstract Initializer

type NaiveInitializer <: Initializer
	length::Float64
	mu::Vector{Float64}
	Sigma::Matrix{Float64}

	function NaiveInitializer(m::SearchDomain)
		mu = m.length/2 * ones(2)
		S = 1e7 * eye(2)
		return new(m.length, mu, S)
	end
end

initialize(ni::NaiveInitializer, x::Vehicle, o::Float64) = true


type LSInitializer <: Initializer
	length::Float64
	mu::Vector{Float64}
	Sigma::Matrix{Float64}
	obs::Vector{Float64}
	pos::Vector{NTuple{2,Float64}}
	min_obs_num::Int

	function LSInitializer(m::SearchDomain)
		return LSInitializer(m.length)
	end
	function LSInitializer(length::Float64)
		S = 3e1*eye(2)
		obs = Array(Float64,0)
		pos = Array(NTuple{2,Float64},0)
		min_obs_num = 5
		return new(length, zeros(2), S, obs, pos, min_obs_num)
	end
end
function initialize(lsi::LSInitializer, x::Vehicle, o::Float64)
	push!(lsi.pos, (x.x,x.y))
	push!(lsi.obs, o)
	if length(lsi.obs) < lsi.min_obs_num
		return false
	end
	# ok, do our triangulation/least squares stuff
	best_sum = Inf
	for xj in linspace(0.0, lsi.length)
		for yj in linspace(0.0, lsi.length)
			square_sum = 0.0
			for i = 1:length(lsi.pos)
				ot = true_bearing(lsi.pos[i], (xj,yj))
				square_sum += fit_180(ot - lsi.obs[i])^2
			end
			if square_sum < best_sum
				best_sum = square_sum
				lsi.mu[1] = xj
				lsi.mu[2] = yj
			end
		end
	end
	return true
end

function check_initialization(gf::GaussianFilter, x::Vehicle, o::Float64)
	gf.initialized = initialize(gf.initializer, x, o)
	if gf.initialized
		gf.mu = gf.initializer.mu
		gf.Sigma = gf.initializer.Sigma
	end
	return gf.initialized
end
