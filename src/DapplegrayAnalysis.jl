module DapplegrayAnalysis

using DapplegrayDynamics
import DapplegrayDynamics as DD
using CoordinateTransformations
using GeometryBasics
using GLMakie
using MeshCat
using MeshCatMechanisms
using Rotations
using StaticArrays

export fj, jf, df, nl, kj, plot_pendulum_iterations

function load_acrobot()::String
    srcdir = dirname(pathof(DapplegrayAnalysis))
    urdf = joinpath(srcdir, "..", "test", "urdf", "Acrobot.urdf")
    urdf
end

function load_pendulum()::String
    srcdir = dirname(pathof(DapplegrayAnalysis))
    urdf = joinpath(srcdir, "..", "test", "urdf", "pendulum.urdf")
    urdf
end

function fj()
    urdf = load_acrobot()
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

function jf()
    urdf = load_pendulum()
    mechanism = parse_urdf(urdf, gravity=SVector(0.0, 0.0, -9.81))

    # Run the optimization
    result = DD.pendulum_swingup_nlopt(mechanism, 51, 10.0, 100)

    # Get the discrete trajectory from the result
    discrete_trajectory = result.solution
    ts = DD.time(discrete_trajectory)
    qs = DD.position_trajectory(discrete_trajectory)

    println("Time steps: ", length(ts))
    println("First position: ", qs[1])
    println("Last position: ", qs[end])

    # Create visualization
    vis = Visualizer()
    mvis = MechanismVisualizer(mechanism, URDFVisuals(urdf), vis)
    animation = Animation(mvis, ts, qs)
    setanimation!(mvis, animation)
    open(vis)
end

function plot_pendulum_iterations(primal_solutions::Vector; max_iterations::Int = 10)
    # Create a figure for plotting all trajectories
    fig = Figure(size = (800, 800))
    ax1 = Axis(
        fig[1, 1],
        xlabel = "θ (theta) [deg]",
        ylabel = "θ̇ (thetadot) [deg/s]",
        title = "Pendulum Phase Portrait",
    )

    ax2 = Axis(
        fig[2, 1],
        xlabel = "Time [s]",
        ylabel = "Control (τ) [Nm]",
        title = "Control Trajectories",
    )

    # Subsample iterations if there are too many
    n_total = length(primal_solutions)
    if n_total <= max_iterations
        indices_to_plot = 1:n_total
    else
        # Plot first, last, and evenly spaced intermediate iterations
        indices_to_plot =
            unique([1; round.(Int, LinRange(2, n_total-1, max_iterations-2)); n_total])
    end

    # Plot each solution trajectory
    for idx ∈ indices_to_plot
        solution_trajectory = primal_solutions[idx]
        ts = DD.time(solution_trajectory)
        qs = DD.position_trajectory(solution_trajectory)
        vs = DD.velocity_trajectory(solution_trajectory)
        us = DD.control_trajectory(solution_trajectory)

        # Extract theta and thetadot for plotting (convert to degrees)
        theta = [rad2deg(first(q)) for q ∈ qs]
        thetadot = [rad2deg(first(v)) for v ∈ vs]

        # Extract controls
        controls = [first(u) for u ∈ us]

        # Plot the phase portrait
        scatterlines!(ax1, theta, thetadot, label = "Iteration $idx")

        # Plot the control trajectory (note: controls have length N-1)
        lines!(ax2, ts[1:length(controls)], controls, label = "Iteration $idx")
    end

    axislegend(ax1, position = :rt)
    axislegend(ax2, position = :rt)
    display(fig)

    return fig
end

function df(urdf::Bool = true)
    mechanism = urdf ? DD.load_acrobot() : DD.doublependulum()
    acrobot_swingup(mechanism, 50, 10.0)
end

function nl()
    mechanism = DD.load_pendulum()
    result = DD.pendulum_swingup_nlopt(mechanism, 51, 10.0, 100)
    plot_pendulum_iterations(result.primal_solutions)
    result
end

function kj()
    mechanism = DD.load_pendulum()
    solver = DD.pendulum_swingup(mechanism, 51, 10.0, 10)

    println(
        "********************************** PRINT GUTS **********************************",
    )
    println(
        "********************************************************************************",
    )
    for (k, _v) in solver.guts
        println(k)
    end
    primal_solutions = solver.guts[:primal]

    plot_pendulum_iterations(primal_solutions)

    solver
end

end
