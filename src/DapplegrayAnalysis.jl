module DapplegrayAnalysis

using Colors
using CoordinateTransformations
using DapplegrayDynamics
using GeometryBasics
using MeshCat
using RobotDynamics: AbstractModel, RBState, AbstractTrajectory, states
using RobotZoo
using Rotations
using StaticArrays
using TrajectoryOptimization

export visualize_swingup

"""
Animate the trajectory of a rigid body in MeshCat
    Assumes the robot geometry is already loaded into `vis["robot"]`
"""
visualize!(vis, model::AbstractModel, Z::AbstractTrajectory) =
    visualize!(vis, model, Z[end].t, states(Z))

"""
    visualize!(vis, model, tf, Xs...; kwargs...)

Visualize many different trajectories of the same model.

# Arguments
* `probs`: any struct that supports `TO.get_model` and `TO.get_trajectory`
* `Zs`: any `AbstractTrajectory`
* `Xs`: a vector of state vectors
"""
function visualize!(vis, model::AbstractModel, tf::Real, Xs::Vector)
    N = length(Xs)
    fps = Int(floor((N-1)/tf))
    anim = MeshCat.Animation(vis, fps=fps)
    for k = 1:N
        atframe(anim, k) do
            robot = vis
            visualize!(robot, model, Xs[k])
        end
    end
    setanimation!(vis, anim)
end

function visualize!(vis, model::RobotZoo.Pendulum, x::StaticVector)
    θ = x[1]
    q = expm((pi-θ) * @SVector [1,0,0])
    settransform!(vis["geom"], LinearMap(QuatRotation(q)))
end

# Pendulum
function _set_mesh!(vis, model::RobotZoo.Pendulum; length=model.len)
    hinge = Cylinder(Point3d(0.05,0,0), Point3d(-0.05,0,0), 0.05)
    rod   = Cylinder(Point3d(0,0,0), Point3d(0,0,length), 0.01)
    mass  = HyperSphere(Point3d(0,0,length), 0.05)
    setobject!(vis["geom"]["hinge"], hinge, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["geom"]["rod"  ], rod,   MeshPhongMaterial(color=colorant"blue"))
    setobject!(vis["geom"]["mass" ], mass , MeshPhongMaterial(color=colorant"red"))
end

function visualize_swingup()
    vis = Visualizer()
    open(vis)
    model = RobotZoo.Pendulum()
    _set_mesh!(vis, model)
    solver  = swingup()
    visualize!(vis, model, get_trajectory(solver))
end

end
