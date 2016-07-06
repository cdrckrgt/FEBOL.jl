######################################################################
# trajectory.jl
#
# handles the generation of ergodic trajectories
######################################################################

type TrajectoryManager
	N::Int
	h::Float64
	T::Float64
	Q::Matrix{Float64}
	R::Matrix{Float64}
	q::Float64

	function TrajectoryManager(N::Int, h::Float64)
		tm = new()
		tm.N = N
		tm.h = h
		tm.T = N*h
		tm.Q = eye(2)
		tm.R = eye(2)
		tm.q = 0.1
		return tm
	end
end

# decomposes a set of positions into a set of ck coefficients
function decompose_trajectory(em::ErgodicManager, traj::VVF64, T::Float64)
	traj2 = [(traj[i][1], traj[i][2]) for i = 1:length(traj)]
	return decompose_trajectory(em, traj2, T)
end
function decompose_trajectory(em::ErgodicManager, traj::Vector{LocTuple}, T::Float64)
	K = em.K
	N = length(traj)
	h = T / N
	ck = zeros(K+1, K+1)
	for k1 = 0:K
		kpiL1 = k1 * pi / em.L
		for k2 = 0:K
			kpiL2 = k2 * pi / em.L
			hk = em.hk[k1+1, k2+1]
			val = 0.0
			# now loop over time
			for p in traj
				val += cos(kpiL1 * p[1])  * cos(kpiL2 * p[2])
			end
			ck[k1+1, k2+1] = (h / T) * val / hk
		end
	end
	return ck
end


# Computes the ergodicity of a decomposed trajectory
# This is the ergodic metric
function score(em::ErgodicManager, traj::Vector{NTuple{2,Float64}}, T::Float64)
	ck = decompose_trajectory(em, traj, T)
	return score(em, ck)
end
function score(em::ErgodicManager, traj::VVF64, T::Float64)
	ck = decompose_trajectory(em, traj, T)
	return score(em, ck)
end
function score(em::ErgodicManager, ck::Matrix{Float64})
	val = 0.0
	for k1 = 0:em.K
		for k2 = 0:em.K
			d = em.phik[k1+1,k2+1] - ck[k1+1,k2+1]
			val += em.Alphak[k1+1,k2+1] * d * d
		end
	end
	return val
end


######################################################################
# Creating discrete control policies
# Right now, I assume simple integrator
######################################################################
# assuming Q_n = Q
# assuming R_n = R
# assuming A_n = A
# assuming B_n = B
function gains_and_descent(em::ErgodicManager, A::Matrix{Float64}, B::Matrix{Float64}, N::Int, xd::VVF64)
	Q = eye(2)
	P = eye(2)
	R = eye(2)

	# TODO: the below is laziness... clean it up
	Ap = A'
	Bp = B'

	# compute an
	a_arr = Array(Vector{Float64}, N+1)

	Karr = Array(Matrix{Float64}, N)
	Parr = Array(Matrix{Float64}, N+1)
	Parr[N+1] = Q
	rarr = Array(Vector{Float64}, N+1)
	varr = Array(Vector{Float64}, N+1)
	zarr = Array(Vector{Float64}, N+1)

	#rarr[N+1] = aN
	rarr[N+1] = compute_an(em, xd, N, N)

	for n = (N-1):-1:0
		G = R + Bp*Parr[n+1+1]*B

		K = inv(G)*Bp*Parr[n+1+1]*B
		Karr[n+1] = K
		Parr[n+1] = Q + Ap*Parr[n+1+1]*A - K'*G*K

		# computing r_n, needed for descent direction
		an = compute_an(em, xd, N, n)
		bn = compute_bn()		# TODO: make this legit
		rarr[n+1] = (A - K'*Bp)*rarr[n+1+1] + an - K'*bn
	end

	# compute descent direction for position and velocity
	# now that we've computed rarr, we need to compute v and z
	zn = [0.0;0.0]
	zarr[1] = zn
	for n = 0:(N-1)
		bn = compute_bn()
		vn = bn + Bp*Parr[n+1]*zn + Bp*rarr[n+1+1]
		zn1 = A*zarr[n+1] + B*vn

		# TODO: are there gonna be deepcopy issues here?
		varr[n+1] = vn
		zarr[n+1+1] = zn1
	end
	varr[N+1] = varr[N]		# last control input doesn't matter (no effect)

	return Karr, zarr, varr
end

# X is a trajectory
#function compute_an(em::ErgodicManager, X::Vector{LocTuple}, N::Int, n::Int)
function compute_an(em::ErgodicManager, xd::VVF64, N::Int, n::Int)
	xnx = xd[n+1][1]
	xny = xd[n+1][2]
	L = em.L
	an = zeros(2)

	h = 0.5	  # TODO: input parameter
	T = h*N	  # TODO: input parameter (is this just h*N?)

	for k1 = 0:em.K
		for k2 = 0:em.K
			dFk_dxn1 = -k1*pi*sin(k1*pi*xnx/L)*cos(k2*pi*xny/L) / (h*L)
			dFk_dxn2 = -k2*pi*cos(k1*pi*xnx/L)*sin(k2*pi*xny/L) / (h*L)

			fk = 0.0
			for i = 1:(N-1)
				x = xd[i]
				fk += cos(k1*pi*x[1]/L)*cos(k2*pi*x[2]/L) / em.hk[k1+1,k2+1]
			end
			c = em.Alphak[k1+1,k2+1] * (fk/T - em.phik[k1+1,k2+1])
			an[1] += c*dFk_dxn1
			an[2] += c*dFk_dxn2
		end
	end
	return an
end

function compute_bn()
	return zeros(2)
end

# h is really like dt between steps here (assume constant steps)
function create_trajectory(em::ErgodicManager, N::Int, h::Float64, x0::LocTuple)

	# initialize trajectory
	xd, ud = initialize_trajectory(N, h, x0)

	# generate linearized dynamics (assumed constant)
	# TODO: generate these from system dynamics
	A = eye(2)
	B = h*eye(2)

	# TODO: really a termination condition
	for i = 1:100
		K, zd, vd = gains_and_descent(em, A, B, N, xd)
		step_size = armijo_ls()

		# Create alpha, mu
		for i = 1:N
			step_size * zd[i][1]
			step_size * zd[i][2]
		end

		project!(xd, ud, zd, vd, step_size, K, N, h)
	end
	return xd, ud
end


# currently, I just move in a random direction
# perhaps find direction to mean and move towards it
function initialize_trajectory(N::Int, h::Float64, x0::LocTuple)
	xd = [[x0[1], x0[2]] for i = 1:N+1]
	ud = [ones(2) for i = 1:N+1]
	for i = 2:N+1
		xd[i][1] = xd[i-1][1] + h*ud[i-1][1]
		xd[i][2] = xd[i-1][2] + h*ud[i-1][2]
	end
	return xd, ud
end


# TODO: actually do this
function armijo_ls()
	return 1.0
end

function project!(xd::VVF64, ud::VVF64, zd::VVF64, vd::VVF64, step_size::Float64, K::VMF64, N::Int, h::Float64)
	for i = 0:(N-1)
		ud[i+1] = ud[i+1] + step_size*vd[i+1]  + K[i+1]*step_size*zd[i+1]
		xd[i+1+1] = xd[i+1] + h*ud[i+1]
	end
	ud[N+1] = ud[N]
end
