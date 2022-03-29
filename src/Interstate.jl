module Interstate

using Colors
using GLMakie
using Polyhedra
using SparseArrays
using StaticArrays
using Printf
using OSQP

include("movables.jl")
include("sensors.jl")
include("world.jl")
include("simulate.jl")
include("fleet_control.jl")
include("keyboard_control.jl")

export Movable, Unicycle, Bicycle, VehicleControl, Oracle, FleetOracle, OracleMeas, Simulator, simulate, keyboard_broadcaster, get_corners, speed, heading, controller, random_road, simple_loop, visualize_road, fleet_controller

end
