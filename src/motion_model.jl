# TODO: prevent motion outside of domain

abstract type MotionModel end

struct NoMotion <: MotionModel
end
move_target(sm::NoMotion, theta, L::Float64) = theta


struct RandomMotion <: MotionModel
    d::Float64
end
RandomMotion() = RandomMotion(5.0)

function move_target(rm::RandomMotion, theta, L::Float64)
    ang = rand() * 2.0 * pi
    new_x = theta[1] + cos(ang) * rm.d
    new_y = theta[2] + sin(ang) * rm.d
    return theta[1] + rm.d*rand(), theta[2] + rm.d*rand(), 0.0, 0.0
end

struct ConstantMotion <: MotionModel
    noise_vals::NTuple{4,Float64}   # std deviations for each dimension
end
ConstantMotion() = ConstantMotion((1.0,1.0,0.05,0.05))
#ConstantMotion(a::Float64, b::Float = ConstantMotion((a,b,c,d))

function move_target(cm::ConstantMotion, theta, L::Float64)
    #new_x = theta[1] + theta[3]# + cm.noise_vals[1] * randn()
    #new_y = theta[2] + theta[4]# + cm.noise_vals[2] * randn()
    #new_xdot = theta[3]# + cm.noise_vals[3] * randn()
    #new_ydot = theta[4]## + cm.noise_vals[4] * randn()

    new_x = theta[1] + theta[3] + cm.noise_vals[1] * randn()
    new_y = theta[2] + theta[4] + cm.noise_vals[2] * randn()
    new_xdot = theta[3] + cm.noise_vals[3] * randn()
    new_ydot = theta[4] + cm.noise_vals[4] * randn()

    return new_x, new_y, new_xdot, new_ydot
end
