module DapplegrayAnalysis

using DapplegrayDynamics
import DapplegrayDynamics as DD
using Colors
using CoordinateTransformations
using GeometryBasics
using MeshCat
using MeshCatMechanisms
using Rotations
using StaticArrays

export kj

function get_acrobot_urdf()::String
    srcdir = dirname(pathof(DapplegrayAnalysis))
    urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
    urdf
end

function kj()
    urdf = get_acrobot_urdf()
    mechanism = parse_urdf(urdf, gravity=SVector(0.0, 0.0, -9.81))
    sqp_solver = DD.acrobot_swingup(mechanism, 50, 10.0)

    discrete_trajectory = DD.primal(sqp_solver)
    ts = DD.time(discrete_trajectory)
    qs = DD.position_trajectory(discrete_trajectory)
    print("position")
    println(qs)
    vs = DD.velocity_trajectory(discrete_trajectory)
    print("velocity")
    println(vs)
    us = DD.control_trajectory(discrete_trajectory)
    print("control")
    println(us)

    vis = Visualizer()
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf), vis)
    animation = Animation(mvis, ts, qs)
    setanimation!(mvis, animation)
    open(vis)
end

end
