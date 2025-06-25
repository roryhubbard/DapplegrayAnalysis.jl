module DapplegrayAnalysis

import DapplegrayDynamics as DD
using Colors
using CoordinateTransformations
using GeometryBasics
using MeshCat
using Rotations
using StaticArrays

export kj

function kj()
    vis = Visualizer()
    sqp_solver = DD.df()
    discrete_trajectory = DD.primal(sqp_solver)
    ts = DD.time(discrete_trajectory)
    qs = DD.knotpoints(discrete_trajectory)
    open(vis)
end

end
