__precompile__()

module FEBOL

using Distributions: Normal, cdf, MvNormal, pdf
using StatsBase: sample
import StatsBase: entropy

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
    FOVn,
    FOV3

export
    AbstractFilter,
    DF,
    EKF,
    EIF,
    UKF,
    PF

export 
    Policy,
    action,

    RandomPolicy,
    SitPolicy,
    GreedyPolicy,
    CirclePolicy,
    SpinPolicy,
    GaussianMPC,
    GreedyPolicy2

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
    MoveAndRotateCost


export step!, batchsim, batchsim2, sim
export act!
export makenorm
export print_belief
export LSInitializer, NaiveInitializer
export my_pdf
export true_bearing
export simulate


const Pose = NTuple{3, Float64}
const Action = NTuple{3, Float64}
const LocTuple = NTuple{2, Float64}
const ObsBin = Int64

export Pose

# math functions
include("math.jl")

# search domain
include("searchdomain.jl")

# Sensing models
include("sensors/sensors.jl")

include("vehicle.jl")

# Filters
include("filters/filters.jl")

# policies
include("infotheoretic.jl")
include("policies/policies.jl")

# Simulation and Plotting
include("simulations/termination.jl")
include("simulations/costs.jl")
include("simulations/simulate.jl")
include("simulations/simunit.jl")
include("simulations/batchsim.jl")

end # module
