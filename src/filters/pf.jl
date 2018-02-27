######################################################################
# pf.jl
#
# starting to migrate over to ParticleFilters.jl
######################################################################
"""
    BoolDistribution
A distribution that provides the probabilities of true or false. 
Can construct with `BoolDistribution(p_true)`.
"""
struct BoolDistribution
    p::Float64 # probability of true
end

import Base: ==
ParticleFilters.pdf(d::BoolDistribution, s::Bool) = s ? d.p : 1.0-d.p

Base.rand(rng::AbstractRNG, d::BoolDistribution) = rand(rng) <= d.p

ParticleFilters.iterator(d::BoolDistribution) = [true, false]

==(d1::BoolDistribution, d2::BoolDistribution) = d1.p == d2.p

Base.hash(d::BoolDistribution) = hash(d.p)

Base.length(d::BoolDistribution) = 2

# assumes target is stationary
struct StationaryModel
    x::Vehicle
end

# s and sp will be LocTuple's

# generate new target position
ParticleFilters.generate_s(sm::Vehicle, s, a, rng::AbstractRNG) = s

#function ParticleFilters.observation(x::Vehicle, a, sp)
#    xp = (x.x, x.y, x.heading)
#    p_true = O(x.sensor, sp, xp, true)
#    return BoolDistribution(p_true)
#end

function ParticleFilters.obs_weight(sm::Vehicle, a, sp, o)
    xp = (sm.x, sm.y, sm.heading)
    return O(sm.sensor, sp, xp, o)
end

mutable struct PF{OL} <: AbstractFilter
    x::Vehicle
    n::Int
    sensor::Sensor
    obs_list::OL
    sirpf::SimpleParticleFilter{Tuple{Float64,Float64}, LowVarianceResampler, MersenneTwister}
    b::ParticleCollection{Tuple{Float64,Float64}}       # particle set
end

function PF(x::Vehicle, n::Int, L::Real)
    sirpf = SimpleParticleFilter{NTuple{2,Float64}, LowVarianceResampler, MersenneTwister}(x, LowVarianceResampler(n), Base.GLOBAL_RNG)
    bv = NTuple{2,Float64}[]
    for i = 1:n
        push!(bv, (L*rand(), L*rand()))
    end
    b = ParticleCollection{NTuple{2,Float64}}(bv)
    return PF(x, n, x.sensor, 0:1, sirpf, b)
end

function update!(df::PF, x::Vehicle, o)
    a = (0,0,0)
    b = update(df.sirpf, df.b, a, o)
    #for i = 1:length(b.particles)
    #    if typeof(b.particles[i]) != NTuple{2,Float64}
    #        println("yelp!")
    #        println("typeof = ", typeof(b.particles[i]))
    #    end
    #end
    #println("typeof b = ", typeof(b))
    df.b = b
end
#
#function centroid(f::PF)
#    wsum = 0.0
#    xmean = 0.0
#    ymean = 0.0
#    for i = 1:f.n
#        wval = f.W[i]
#        xmean += f.X[i][1] * wval
#        ymean += f.X[i][2] * wval
#        wsum += wval
#    end
#    return xmean/wsum, ymean/wsum
#end
#
#function reset!(f::PF)
#    for i = 1:f.n
#        xi = f.length*rand()
#        yi = f.length*rand()
#        f.X[i] = (xi, yi)
#        f.Xnew[i] = (xi, yi)
#    end
#    fill!(f.W, 0.0)
#end
