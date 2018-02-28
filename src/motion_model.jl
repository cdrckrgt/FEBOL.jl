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
end

function move_target(cm::ConstantMotion, theta, L::Float64)
    new_x = theta[1] + theta[3]
    new_y = theta[2] + theta[4]

    return theta[1] + theta[3] + randn(), theta[2] + theta[4] + randn(), theta[3], theta[4]
end
