######################################################################
# ergodicity.jl
# handles stuff needed for ergodicity
######################################################################

type ErgodicManager
	K::Int
	bins::Int				# number of bins per side
	L::Float64
	cell_size::Float64
	hk::Matrix{Float64}
	phik::Matrix{Float64}

	function ErgodicManager(m::SearchDomain, K::Int)
		hk   = zeros(K+1,K+1)
		phik = zeros(K+1,K+1)
		bins = 10
		L = m.length
		cell_size = L / bins
		em = new(K, bins, L, cell_size, hk, phik)

		compute_hk!(em)
		return em
	end
end

function compute_hk!(em::ErgodicManager)
	for k1 = 0:em.K
		for k2 = 0:em.K
			em.hk[k1+1,k2+1] = hk_ij(em, k1, k2)
		end
	end
end

function hk_ij(em::ErgodicManager, k1::Int, k2::Int)

	half_size = em.cell_size / 2.0
	kpl1 = k1 * pi / em.L
	kpl2 = k2 * pi / em.L
	cs2 = em.cell_size * em.cell_size
	val = 0.0

	# iterate over all bins
	for xi = 1:em.bins
		x = (xi-1)*em.cell_size + half_size
		cx = cos(kpl1 * x)
		cx2 = cx * cx
		for yi = 1:em.bins
			y = (yi-1)*em.cell_size + half_size
			cy = cos(kpl2 * y)

			val += cx2 * cy * cy * cs2
		end
	end

	return sqrt(val)
end

function phik!(em::ErgodicManager, d::Distribution)
	for k1 = 0:em.K
		for k2 = 0:em.K
			em.hk[k1+1,k2+1] = phik_ij(em, k1, k2)
		end
	end
end

# iterate over the state space
function phik_ij(em::ErgodicManager, k1, k2)
	# iterate over all bins
	for xi = 1:em.bins
		x = (xi-1)*em.cell_size + half_size
		cx = cos(kpl1 * x)
		cx2 = cx * cx
		for yi = 1:em.bins
			y = (yi-1)*em.cell_size + half_size
			cy = cos(kpl2 * y)

			val += cx2 * cy * cy * cs2
		end
	end
end
