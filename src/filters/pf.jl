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
# recall that "a" here is the state of the drone
ParticleFilters.obs_weight(m::Model, a::Pose, sp, o) = O(m.sensor, sp, a, o)

# TODO: should this type of thing go in model subtype?
export CircularRange
struct CircularRange
    h_min::Float64      # minimum heading, in degrees
    h_max::Float64      # maximum heading, in degrees
    v_max::Float64      # maximum velocity
end
function Base.rand(c::CircularRange)
    heading = rand()*(c.h_max - c.h_min) + c.h_min
    velocity = rand()*c.v_max

    vx = velocity * sind(heading)
    vy = velocity * cosd(heading)

    return vx, vy
end


mutable struct PF{M <: Model, OL} <: AbstractFilter
    model::M            # contains vehicle, sensor, target motion models
    n::Int              # number of particles
    obs_list::OL        # list of possible observations
    L::Float64          # length of square domain
    vr::CircularRange   # range of possible velocities

    # User should never see this
    _pf::SimpleParticleFilter{TargetTuple, M, LVR, MerTwist}
    _b::ParticleCollection{TargetTuple}       # particle set
end

function PF(model::Model, n::Int, obs_list=0:0;
            L::Real=100.0,
            vr::CircularRange=CircularRange(0,360,3)
           )
    pf = SimpleParticleFilter(model, LVR(n), Base.GLOBAL_RNG)
    b = initialize_particles(n, L, vr)

    return PF(model, n, obs_list, L, vr, pf, b)
end

ParticleFilters.state_type(m::Model) = TargetTuple


# TODO: allow user to dictate original spread of velocities
function initialize_particles(n::Int, L::Real, vr::CircularRange)
    bv = TargetTuple[]
    for i = 1:n
        # it was done like the below first
        #push!(bv, (L*rand(), L*rand(), 2.0*rand(), 2.0*rand()))
        vx, vy = rand(vr)
        push!(bv, (L*rand(), L*rand(), vx, vy) )
    end
    return ParticleCollection(bv)
end

# convenience functions
ParticleFilters.particles(pf::PF) = pf._b.particles
ParticleFilters.particle(pf::PF, i::Int) = particle(pf._b, i)
ParticleFilters.weight(pf::PF, i::Int) = weight(pf._b, i)
ParticleFilters.weight_sum(pf::PF) = weight_sum(pf._b)

function predict(pf::PF)
    m = pf.model.motion_model
    return collect( move_target(m , s, 0.0) for s in particles(pf) )
end
function predict(pf::PF, b)
    m = pf.model.motion_model
    return collect( move_target(m , s, 0.0) for s in b.particles )
end

function predict!(pf::PF)
    m = pf.model.motion_model
    particle_vec = collect( move_target(m , s, 0.0) for s in particles(pf) )
    pf._b = ParticleCollection(particle_vec)
end

# vehicle is included in model 
# normally update(filter, belief, a, o), but action not needed
# so I pass in the UAV pose instead
function update!(pf::PF, p::Pose, o)
    pf._b = update(pf._pf, pf._b, p, o)
end

function update_b(pf::PF, b::ParticleCollection, p, o)
    return update(pf._pf, b, p, o)
end
export update_b

function update_bp{S}(up::SimpleParticleFilter{S}, b::ParticleCollection, a, o)
    ps = particles(b)
    pm = up._particle_memory
    wm = up._weight_memory
    resize!(pm, 0)
    resize!(wm, 0)
    sizehint!(pm, n_particles(b))
    sizehint!(wm, n_particles(b))
    for i in 1:n_particles(b)
        s = ps[i]
        push!(pm, s)
        push!(wm, obs_weight(up.model, s, a, s, o))
    end

    return resample(up.resample, WeightedParticleBelief{S}(pm, wm, sum(wm), nothing), up.rng)
end
export update_bp

# TODO: should I pass in something other than Base.GLOBAL_RNG) ?
function sample_state(pf::PF)
    return rand(Base.GLOBAL_RNG, pf._b)
end

# n is the number of states you want to sample
function sample_states(pf::PF, n::Int)
    N = n_particles(pf._b)
    bv = Array{TargetTuple}(n)
    for i = 1:n
        bv[i] = particle(pf, rand(1:N))
    end
    return ParticleCollection(bv)
end

function reset!(f::PF)
    f._b = initialize_particles(f.n, f.L, f.vr)
end

function centroid(pf::PF)
    mx = 0.0
    my = 0.0
    for i = 1:pf.n
        tp = particle(pf, i)
        mx += tp[1]
        my += tp[2]
    end

    return mx/pf.n, my/pf.n
end
function centroid(b::ParticleCollection)
    mx = 0.0
    my = 0.0
    n = length(b.particles)
    for i = 1:n
        tp = particle(b, i)
        mx += tp[1]
        my += tp[2]
    end

    return mx/n, my/n
end

function covariance(pf::PF)
    xx = 0.0
    xy = 0.0
    yy = 0.0

    mx = 0.0
    my = 0.0
    for i = 1:pf.n
        tp = particle(pf, i)

        xx += tp[1] * tp[1]
        xy += tp[1] * tp[2]
        yy += tp[2] * tp[2]

        mx += tp[1]
        my += tp[2]
    end
    xx /= pf.n
    xy /= pf.n
    yy /= pf.n

    mx /= pf.n
    my /= pf.n

    a = xx - mx * mx
    bc = xy - mx * my
    d = yy - my * my

    return [a bc; bc d]
end


export bin_pf
function bin_pf(b::ParticleCollection, discrete_b::Matrix{Float64}, L::Float64)

    # determine the size of each cell
    n_cells = size(discrete_b, 1)        # assuming symmetric matrix
    cell_size = L / n_cells

    # ensure the matrix is filled with zeros
    fill!(discrete_b, 0.0)

    # loop over particles and determine where they belong
    n = n_particles(b)
    w = 1.0 / n
    for i = 1:n
        tp = particle(b, i)

        xi = round(Int, tp[1] / cell_size, RoundUp)
        yi = round(Int, tp[2] / cell_size, RoundUp)

        # ensure that cells are legal
        xi = min(max(xi, 1), n_cells)
        yi = min(max(yi, 1), n_cells)

        discrete_b[xi, yi] += w
    end
end


function cheap_entropy(b::ParticleCollection,m::Matrix{Float64},L::Float64)

    # bin the particle collection into a matrix
    bin_pf(b, m, L)

    # now compute entropy from the matrix
    ent = 0.0
    for p in m
        if p > 0.0
            ent -= p * log(p)
        end
    end

    return ent
end

export cheap_entropy

function cheap_entropy(pf::PF, L::Float64, n_cells::Int)
    cheap_entropy(pf._b, L, n_cells)
end
function cheap_entropy(b::ParticleCollection, L::Float64, n_cells::Int)
    discrete_b = zeros(n_cells, n_cells)
    return cheap_entropy(b, discrete_b, L)
end
