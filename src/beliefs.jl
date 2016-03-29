abstract Belief

type Gaussian <: Belief
	mean::Vector{Float64}
	Sigma::Matrix{Float64}
end

type KF
end

function update!(b::Gaussian, kf::KF)
end
