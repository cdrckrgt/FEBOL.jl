######################################################################
# eif.jl
# Extended information filter
######################################################################

type EIF
	eta::Vector{Float64}
	Omega::Matrix{Float64}
	theta::NTuple{2, Float64}

	function EIF(m::SearchDomain)
		Omega = 1e-8 * eye(2)
		eta = inv(Omega) * ones(2) * m.length / 2

		return new(eta, Omega, m.theta)
	end
end


# have to include search domain because jammer location factors in
# Really, I should just fold that into the state
# TODO: just subtracting is not ok, need circle distance (can be negative)
function update!(eif::EIF, x::Vehicle, o::Float64)
	if eif.Omega == zeros(2,2)
		eif.Omega = [1e-8 0; 0 1e-8]
	end
	mu_t = inv(eif.Omega) * eif.eta

	# compute Omega_t
	xr = m.theta[1] - x.x
	yr = m.theta[2] - x.y
	Ht = [yr, -xr]' * (180.0 / pi) / (xr^2 + yr^2)
	Omega_t = eif.Omega + dot(vec(Ht),vec(Ht)) / 100.0

	# compute eta_t
	eta_t = eif.eta + Ht' * (o - true_bearing((x.x,x.y),eif.theta) + Ht*mu_t) / 100.0

	eif.eta = vec(eta_t)
	eif.Omega = Omega_t
end
