module DapplegrayTest

export test_simulate

using Colors: RGBA, RGB, @colorant_str
using StaticArrays: @SVector
using Rotations: QuatRotation, expm, RotX
using CoordinateTransformations: LinearMap
using GeometryBasics: HyperRectangle, Mesh, Point, TriangleFace, Vec, Point3d,
                      Cylinder, HyperSphere
using MeshCat: Visualizer, setobject!, MeshPhongMaterial, setanimation!,
               settransform!, Animation, atframe

using DapplegrayDynamics: pendulum, simulate_pendulum

# Pendulum
function _set_mesh!(vis, length)
    hinge = Cylinder(Point3d(0.05, 0, 0), Point3d(-0.05, 0, 0), 0.05)
    rod   = Cylinder(Point3d(0, 0, 0), Point3d(0, 0, length), 0.01)
    mass  = HyperSphere(Point3d(0, 0, length), 0.05)
    setobject!(vis["geom"]["hinge"], hinge, MeshPhongMaterial(color=colorant"black"))
    setobject!(vis["geom"]["rod"  ], rod,   MeshPhongMaterial(color=colorant"blue"))
    setobject!(vis["geom"]["mass" ], mass , MeshPhongMaterial(color=colorant"red"))
end

function test_simulate()
    vis = Visualizer()
    open(vis)
    _set_mesh!(vis, pendulum.len)

    sol = simulate_pendulum()

    fps = 1000
    anim = Animation(vis, fps=fps)
    for (tt, uu) in zip(sol.t, sol.u)
        frame = floor(Int, tt * fps)
        atframe(anim, frame) do
            q = expm((π - uu[1]) * @SVector [1,0,0])
            settransform!(vis["geom"], LinearMap(QuatRotation(q)))
        end
    end

    setanimation!(vis, anim)
end

end
