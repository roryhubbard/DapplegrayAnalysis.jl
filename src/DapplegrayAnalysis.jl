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
    mechanism = parse_urdf(urdf)
    sqp_solver = DD.acrobot_swingup(mechanism)

    discrete_trajectory = DD.primal(sqp_solver)
    ts = DD.time(discrete_trajectory)
    qs = DD.knotpoints(discrete_trajectory)

    vis = Visualizer()
#    open(vis)
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf), vis)
    animation = Animation(mvis, ts, qs)
    setanimation!(mvis, animation)
end

end
