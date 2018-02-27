######################################################################
# pf.jl
#
# starting to migrate over to ParticleFilters.jl
######################################################################

# assumes target is stationary
struct StationaryModel
    x::Vehicle
end

# s and sp will be LocTuple's

# generate new target position
ParticleFilters.generate_s(sm::Vehicle, s, a, rng::AbstractRNG) = s

function ParticleFilters.obs_weight(sm::Vehicle, a, sp, o)
    xp = (sm.x, sm.y, sm.heading)
    return O(sm.sensor, sp, xp, o)
end

mutable struct PF <: AbstractFilter
    x::Vehicle
    n::Int
    sirpf::SIRParticleFilter
    b       # particle set
end

function PF(x::Vehicle, n::Int, L::Real)
    sirpf = SIRParticleFilter(x, n)
    bv = NTuple{2,Float64}[]
    for i = 1:n
        push!(bv, (float(L*rand()), float(L*rand())))
    end
    b = ParticleCollection(bv)
    return PF(x, n, sirpf, b)
end

function update!(df::PF, x::Vehicle, o)
    a = (0,0,0)
    df.b = update(df.sirpf, df.b, a, o)
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
