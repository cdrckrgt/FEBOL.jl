######################################################################
# pf.jl
#
# starting to migrate over to ParticleFilters.jl
######################################################################

# generate new target position
#ParticleFilters.generate_s(sm::Vehicle, s, a, rng::AbstractRNG) = s

# To shorten some of these names
const LVR = LowVarianceResampler
const MerTwist = MersenneTwister
const SPF = SimpleParticleFilter{TargetTuple, LVR, MerTwist}

struct Model{V <: Vehicle, S <: Sensor, M <: MotionModel}
    x::V
    sensor::S
    motion_model::M
end
Model(x::Vehicle) = Model(x, x.sensor)
Model(x::Vehicle, s::Sensor) = Model(x, s, NoMotion())

function ParticleFilters.generate_s(m::Model, s, a, rng::AbstractRNG)
    return move_target(m.motion_model, s, 0.0)
end
# recall that a here is the state of the drone
function ParticleFilters.obs_weight(m::Model, a::Pose, sp, o)
    return O(m.sensor, sp, a, o)
end


mutable struct PF{M <: Model, OL} <: AbstractFilter
    model::M        # contains vehicle, sensor, target models
    n::Int          # number of particles
    obs_list::OL    # list of possible observations:

    # User should never see this
    _pf::SimpleParticleFilter{TargetTuple, LVR, MerTwist}
    _b::ParticleCollection{TargetTuple}       # particle set
end

function PF(model::Model, n::Int, obs_list=0:0; L::Real=100.0)
    pf = SPF(model, LVR(n), Base.GLOBAL_RNG)
    b = initialize_particles(n, L)

    return PF(model, n, obs_list, pf, b)
end


# TODO: allow user to dictate original spread of velocities
function initialize_particles(n::Int, L::Real)
    bv = TargetTuple[]
    for i = 1:n
        push!(bv, (L*rand(), L*rand(), 2.0*rand(), 2.0*rand()))
    end
    return ParticleCollection(bv)
end

# convenience functions
ParticleFilters.particles(pf::PF) = pf._b.particles
ParticleFilters.particle(pf::PF, i::Int) = particle(pf._b, i)
ParticleFilters.weight(pf::PF, i::Int) = weight(pf._b, i)
ParticleFilters.weight_sum(pf::PF) = weight_sum(pf._b)

# vehicle is included in model 
# normally update(filter, belief, a, o), but action not needed
# so I pass in the UAV pose instead
function update!(pf::PF, p::Pose, o)
    pf._b = update(pf._pf, pf._b, p, o)
end

# TODO: should I pass in something other than Base.GLOBAL_RNG) ?
export sample_state
function sample_state(pf::PF)
    return rand(Base.GLOBAL_RNG, pf._b)
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

function centroid(pf::PF)
    mx = 0.0
    my = 0.0
    w_sum = 0.0
    for i = 1:pf.n
        tp = particle(pf, i)
        w = weight(pf, i)
        mx += w * tp[1]
        my += w * tp[2]
        w_sum += w
    end
    mx /= w_sum
    my /= w_sum

    return (mx, my)
end
function covariance(pf::PF)
    xx = 0.0
    xy = 0.0
    yy = 0.0

    mx = 0.0
    my = 0.0
    w_sum = 0.0
    for i = 1:pf.n
        tp = particle(pf, i)
        w = weight(pf, i)

        xx += w * tp[1] * tp[1]
        xy += w * tp[1] * tp[2]
        yy += w * tp[2] * tp[2]

        mx += w * tp[1]
        my += w * tp[2]

        w_sum += w
    end
    xx /= w_sum
    xy /= w_sum
    yy /= w_sum
    mx /= w_sum
    my /= w_sum

    a = xx - mx * mx
    bc = xy - mx * my
    d = yy - my * my

    return [a bc; bc d]
end
