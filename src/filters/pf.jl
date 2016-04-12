######################################################################
# pf.jl
# Basic filter described in Chapter 4 of Probo Robo
######################################################################


type PF <: AbstractFilter
	n::Int    # number of particles
	X::Vector{LocTuple}
	W::Vector{Float64}
	Xnew::Vector{LocTuple}

	function PF(m::SearchDomain, n::Int)
		X = Array(LocTuple, n)
		for i = 1:n
			xi = m.length*rand()
			yi = m.length*rand()
			X[i] = (xi, yi)
		end
		Xnew = deepcopy(X)
		W = zeros(n)
		return new(n, X, W, Xnew)
	end
end

function update!(pf::PF, x::Vehicle, o::Float64)
	w_sum = 0.0
	for i = 1:pf.n
		# don't need to sample from transition, just calculate weights
		 pf.W[i] = O(x, pf.X[i], o) # calculate probability of obs
		 w_sum = pf.W[i]
	end

	for i = 1:pf.n
		# draw with probability
		pf.Xnew[i] = sample(pf.X, WeightVec(pf.W))
	end
	copy!(pf.X, pf.Xnew)
end
