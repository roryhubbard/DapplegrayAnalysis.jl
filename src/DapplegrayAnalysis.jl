module DapplegrayAnalysis

using Colors
using CoordinateTransformations
using DapplegrayDynamics
using GeometryBasics
using MeshCat
using Rotations
using StaticArrays

export visualize_doublependulum

function visualize!(vis, model::Symbol, ts::Vector, qs::Vector, args...; kwargs...)
    N = length(ts)
    fps = Int(floor((N-1)/(ts[end] - ts[1])))
    anim = MeshCat.Animation(vis, fps=fps)
    for k = 1:N
        atframe(anim, k) do
            robot = vis
            visualize!(robot, Val(model), qs[k], args...; kwargs...)
        end
    end
    setanimation!(vis, anim)
end

function visualize!(vis, ::Val{:doublependulum}, x, link1length::Real)
    e1 = @SVector [1,0,0]
    q1,q2 = expm((x[1]+π)*e1), expm(x[2]*e1)
    settransform!(vis["shoulder"], LinearMap(UnitQuaternion(q1)))
    settransform!(vis["shoulder", "upperlink", "elbow"], LinearMap(UnitQuaternion(q2)))
end

function set_mesh!(vis, model::Symbol, args...; kwargs...)
    set_mesh!(vis, Val(model), args...; kwargs...)
end

function set_mesh!(vis, ::Val{:doublependulum}, link1length, link2length; color=colorant"blue", thick=0.05)
    hinge = Cylinder(Point3d(-thick/2.0,0,0), Point3d(thick/2.0,0,0), thick)
    dim1  = Vec(thick, thick, link1length)
    upperlink = Rect3d(Vec(-thick/2,-thick/2,0),dim1)
    dim2  = Vec(thick, thick, link2length)
    lowerlink = Rect3d(Vec(-thick/2,-thick/2,0),dim2)
    mat1 = MeshPhongMaterial(color=colorant"grey")
    mat2 = MeshPhongMaterial(color=color)
    setobject!(vis["shoulder"], hinge, mat1)
    setobject!(vis["shoulder", "upperlink"], upperlink, mat2)
    setobject!(vis["shoulder", "upperlink","elbow"], hinge, mat1)
    setobject!(vis["shoulder", "upperlink","elbow","lowerlink"], lowerlink, mat2)
    settransform!(vis["upperlink"], Translation(0,0,-link1length))
    e1 = @SVector [1,0,0]
    q1 = expm(π*e1)
    settransform!(vis["shoulder"], LinearMap(UnitQuaternion(q1)))
    settransform!(vis["shoulder", "upperlink","elbow"], Translation(0,0,link1length))
end

function visualize_doublependulum()
    vis = Visualizer()
    link1length = 1.0
    link2length = 1.0
    set_mesh!(vis, :doublependulum, link1length, link2length)
    ts, qs, vs = doublependulum()
    visualize!(vis, :doublependulum, ts, qs, link1length)
    open(vis)
end

end
