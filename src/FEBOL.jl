__precompile__()

module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using StatsBase: sample
import StatsBase: entropy
using ParticleFilters

export
    SearchDomain,
    theta!

export
    Vehicle,
    new_pose

export
    Sensor,
    observe,
    update!,
    centroid,
    covariance,
    entropy,
    reset!,

    RangeOnly,
    BearingOnly,
    DirOmni,
    FOV,
    FOVb,
    FOVn,
    FOV3

export
    AbstractFilter,
    DF,
    EKF,
    EIF,
    UKF,

    PF,
    particle,
    particles,
    predict,
    ParticleCollection,
    sample_state,
    sample_states

export 
    Policy,
    action,

    RandomPolicy,
    SitPolicy,
    GreedyPolicy,
    GreedyP,
    CirclePolicy,
    SpinPolicy,
    GaussianMPC,
    GreedyPolicy2,

    make_action_list

export
    SimUnit,

    TerminationCondition,
    is_complete,
    StepThreshold,
    MaxNormThreshold,

    CostModel,
    get_cost,
    ConstantCost,
    MoveCost,
    MoveCost2,
    MoveAndRotateCost,

    step!,
    simulate,
    parsim

export
    Model,
    MotionModel,
    StationaryMotion,
    RandomMotion,
    ConstantMotion,
    move_target!,
    move_target


export act!
export makenorm
export print_belief
export LSInitializer, NaiveInitializer
export my_pdf
export true_bearing


const Pose = NTuple{3, Float64}
const Action = NTuple{3, Float64}
const LocTuple = NTuple{4, Float64}
const TargetTuple = NTuple{4, Float64}
const ObsBin = Int64

export TargetTuple

export Pose

# math functions
include("math.jl")

# search domain
include("motion_model.jl")
include("search_domain.jl")

# Sensing models
include("sensors/sensors.jl")

include("vehicle.jl")

# Filters
include("filters/filters.jl")

# information theory
include("information_theory/mutual_information.jl")
include("information_theory/fisher.jl")
include("information_theory/cache.jl")
include("information_theory/kl.jl")

# policies
include("policies/policies.jl")

# Simulation and Plotting
include("simulations/termination.jl")
include("simulations/costs.jl")
include("simulations/simunit.jl")
include("simulations/simulate.jl")
include("simulations/parallel.jl")

end # module
