######################################################################
# ergodicity.jl
# handles stuff needed for ergodicity
# TODO: omfg Alphak is really Lambdak
######################################################################

typealias VVF64   Vector{Vector{Float64}}
typealias VMF64   Vector{Matrix{Float64}}

# kpixl is a (K+1 x bins) matrix, each entry storing cos(k*pi*x / L)
#  this assumes some discretization
type ErgodicManager
	K::Int
	bins::Int				# number of bins per side
	L::Float64
	cell_size::Float64
	hk::Matrix{Float64}
	phik::Matrix{Float64}
	Alphak::Matrix{Float64}
	kpixl::Matrix{Float64}

	function ErgodicManager(m::SearchDomain, K::Int, bins::Int)
		hk   = zeros(K+1,K+1)
		phik = zeros(K+1,K+1)
		Alphak = zeros(K+1,K+1)
		kpixl = zeros(K+1, bins)
		L = m.length
		cell_size = L / bins
		em = new(K, bins, L, cell_size, hk, phik, Alphak, kpixl)

		Alphak!(em)
		kpixl!(em)
		hk!(em)
		return em
	end
	ErgodicManager(m::SearchDomain) = ErgodicManager(m, 10, 100)
end

# fills the Alpha_{k1,k2}
function Alphak!(em::ErgodicManager)
	for k1 = 0:em.K
		for k2 = 0:em.K
			den = (1.0 + k1*k1 + k2*k2) ^ 1.5
			em.Alphak[k1+1, k2+1] = 1.0 / den
		end
	end
end

# TODO: check if the row/col ordering of this is julia-efficient
function kpixl!(em::ErgodicManager)
	half_size = em.cell_size / 2.0
	for xi = 1:em.bins
		x = (xi-1)*em.cell_size + half_size
		for k = 0:em.K
			em.kpixl[k+1,xi] = cos(k*pi*x / em.L)
		end
	end
end

# generates the hk coefficients for the ergodic manager
# these coefficients only need to be computed once
function hk!(em::ErgodicManager)
	for k1 = 0:em.K
		for k2 = 0:em.K
			em.hk[k1+1,k2+1] = hk_ij(em, k1, k2)
		end
	end
end

# computes the coefficients for a specific value of k1 and k2
# called by hk!
function hk_ij(em::ErgodicManager, k1::Int, k2::Int)
	cs2 = em.cell_size * em.cell_size
	val = 0.0
	for xi = 1:em.bins
		cx = em.kpixl[k1+1,xi]
		cx2 = cx * cx
		for yi = 1:em.bins
			cy = em.kpixl[k2+1,yi]
			val += cx2 * cy * cy * cs2
		end
	end

	return sqrt(val)
end

######################################################################
# Computing Fourier coefficients
######################################################################
# update the Fourier coefficients based on some distribution
# Here, I assume it is discrete, but I should change this...
function phik!(em::ErgodicManager, d::Matrix{Float64})
	for k1 = 0:em.K
		for k2 = 0:em.K
			em.phik[k1+1,k2+1] = phik_ij(em, k1, k2, d)
		end
	end
end

function phik!(em::ErgodicManager, dm::Vector{Float64}, ds::Matrix{Float64})
	# first, generate d
	half_size = em.cell_size / 2.0
	d = zeros(em.bins, em.bins)
	d_sum = 0.0
	for xi = 1:em.bins
		x = (xi-1)*em.cell_size + half_size
		for yi = 1:em.bins
			y = (yi-1)*em.cell_size + half_size
			#d[xi,yi] = my_pdf([x,y], dm, ds)
			d[xi,yi] = my_pdf((x,y), dm, ds)
			d_sum += d[xi,yi]
		end
	end

	# normalize... I'm not sure if this is necessary
	for xi = 1:em.bins
		for yi = 1:em.bins
			d[xi,yi] /= d_sum
		end
	end

	phik!(em, d)
end


# iterate over the state space
function phik_ij(em::ErgodicManager, k1, k2, d::Matrix{Float64})
	val = 0.0
	cs2 = em.cell_size * em.cell_size
	for xi = 1:em.bins
		cx = em.kpixl[k1+1,xi]
		for yi = 1:em.bins
			cy = em.kpixl[k2+1,yi]
			val += d[xi,yi] * cx * cy * cs2
		end
	end
	return val / em.hk[k1+1,k2+1]
end

######################################################################
# Reconstructing an underlying function
# TODO: check that I don't have to multiply by cell size squared
######################################################################
function reconstruct(em::ErgodicManager)
	# iterate over all bins
	half_size = em.cell_size / 2.0
	cs2 = em.cell_size * em.cell_size

	vals = zeros(em.bins, em.bins)

	for xi = 1:em.bins
		x = (xi-1)*em.cell_size + half_size
		for yi = 1:em.bins
			y = (yi-1)*em.cell_size + half_size
			for k1 = 0:em.K
				cx = em.kpixl[k1+1,xi]
				for k2 = 0:em.K
					cy = em.kpixl[k2+1,yi]
					vals[xi,yi] += em.phik[k1+1,k2+1]*cx*cy/em.hk[k1+1,k2+1]
				end
			end
		end
	end
	return vals
end
