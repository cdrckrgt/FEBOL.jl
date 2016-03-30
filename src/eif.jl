######################################################################
# eif.jl
# Extended information filter
######################################################################

type InfoMatrix <: Belief
	eta::Vector{Float64}
	Omega::Matrix{Float64}
end

type EIF
end

function initial_belief(eif::EIF)
	#return InfoMatrix([0.,0.], [0.0 0.0; 0 0.0])
	Omega = [1e-8 0; 0 1e-8]
	eta = inv(Omega) * [5.,5]
	return InfoMatrix(eta, Omega)
end

# have to include search domain because jammer location factors in
# Really, I should just fold that into the state
# TODO: just subtracting is not ok, need circle distance (can be negative)
function update!(m::SearchDomain, b::InfoMatrix, eif::EIF, x::Vehicle, o::Obs)
	if b.Omega == zeros(2,2)
		b.Omega = [1e-8 0; 0 1e-8]
	end
	mu_t = inv(b.Omega) * b.eta

	# compute Omega_t
	xr = m.theta[1] - x.x
	yr = m.theta[2] - x.y
	Ht = [yr, -xr]' * (180.0 / pi) / (xr^2 + yr^2)
	Omega_t = b.Omega + dot(vec(Ht),vec(Ht)) / 100.0

	# compute eta_t
	eta_t = b.eta + Ht' * (o - true_bearing((x.x,x.y),m.theta) + Ht*mu_t) / 100.0

	b.eta = vec(eta_t)
	b.Omega = Omega_t
end
