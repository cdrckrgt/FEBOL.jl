######################################################################
# pf.jl
#
# starting to migrate over to ParticleFilters.jl
######################################################################

# generate new target position
#ParticleFilters.generate_s(sm::Vehicle, s, a, rng::AbstractRNG) = s

# random
#function ParticleFilters.generate_s(sm::Vehicle, s, a, rng::AbstractRNG)
#    ang = rand() * 2.0 * pi
#    return s[1] + 5.0 * cos(ang), s[2] + 5*sin(ang)
#enk

const LVR = LowVarianceResampler

struct Model{S <: Sensor, M <: MotionModel, V <: Vehicle}
    sensor::S
    motion_model::M
    x::V
end

function ParticleFilters.generate_s(m::Model, s, a, rng::AbstractRNG)
    return move_target(m.motion_model, s, 0.0)
end
function ParticleFilters.obs_weight(m::Model, a, sp, o)
    return O(m.sensor, sp, get_pose(m.x), o)
end


mutable struct PF{OL} <: AbstractFilter
    model::Model

    x::Vehicle
    sensor::Sensor

    n::Int

    obs_list::OL


    # User should never see this
    _pf::SimpleParticleFilter{TargetTuple, LVR, MersenneTwister}
    _b::ParticleCollection{TargetTuple}       # particle set
end

function PF(x::Vehicle, n::Int, L::Real)
    model = Model(x.sensor, ConstantMotion(), x)
    pf = SimpleParticleFilter{TargetTuple, LVR, MersenneTwister}(model, LVR(n), Base.GLOBAL_RNG)

    # generate initial source of particles
    bv = TargetTuple[]
    for i = 1:n
        push!(bv, (L*rand(), L*rand(), 2.0*rand(), 2.0*rand()))
    end
    b = ParticleCollection(bv)
    return PF(model, x, x.sensor, n, 0:1, pf, b)
end

ParticleFilters.particles(pf::PF) = pf._b.particles
ParticleFilters.particle(pf::PF, i::Int) = particle(pf._b, i)
ParticleFilters.weight(pf::PF, i::Int) = weight(pf._b, i)
ParticleFilters.weight_sum(pf::PF) = weight_sum(pf._b)

function update!(pf::PF, x::Vehicle, o)
    a = (0,0,0)
    pf._b = update(pf._pf, pf._b, a, o)
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
