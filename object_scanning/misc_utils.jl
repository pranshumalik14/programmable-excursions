"""
Miscellaneous functions to facilitate generation and plotting of a smooth scanning profile
around the object, preferably a constant distance away from the surface and accounting for
other objects/obstacles in the way.
"""

using Plots
using Parameters


"""
Types and struct definitions for holding map and pose information
"""

abstract type GeometricEntity end
abstract type AbstractPose <: GeometricEntity end
abstract type AbstractPoint <: GeometricEntity end

# holds 2D map and its properties to hold the (sliced) object and environment information
@with_kw mutable struct Map
    map::Matrix{Real}
    res::Real
    low::Real
    high::Real
    und::Real
end

# represents a 2D frame, (implicitly) treating it as a pose wrt world frame. this entity is
# immutable.
@with_kw struct Frame2 <: AbstractPose
    x::Real
    y::Real
    θ::Real
    name::AbstractString
end

# constant global struct for world coordinate frame
const 𝑊 = Frame2(0.0, 0.0, 0.0, "World")

# holds 2D point information; a vector
@with_kw mutable struct Point2 <: AbstractPoint
    x::Real
    y::Real
    𝑢::Frame2 = 𝑊
end

# representation of 2D relative pose, frame {𝑢} wrt frame {𝑣} or rigid body motion from
# {𝑢} to {𝑣}. note that, by convention, θ increases in the anticlockwise direction. the
# default frame is the world frame. this entity lives in SE(2).
@with_kw mutable struct Pose2 <: AbstractPose
    x::Real
    y::Real
    θ::Real
    𝑢::Frame2 = 𝑊
    𝑣::Frame2 = 𝑊
end


"""
Pose operations
"""

# translate (SE2)
# rotate (SE2)
# frame change, pose diff, pose add, pose struct
# ᵛp = ᵛξᵤ ⋅ ᵘp -> dot operator for Pose to point. Will assert base frames to be the same
# ᵛξₜ = ᵛξᵤ ⊕ ᵘξₜ -> oplus operator for pose to pose. Will assert base frames to be the same
# minus operator on pose and point2, relying on the unary - operator defined for both


"""
Heuristic evaluation utils
"""

# return heuristic per object point
# return heuristic points for collection
# heuristic evaluation over map

# mutates free space elements (low val)
function evaluate_heuristic()
    #
end


"""
map setup utils
"""

# returns a matrix of size w/res × l/res, filling elements between and including the two
# scalar functions f and g with the high value. rest of the map above f is filled with low
# value and below g is filled with the undefined value. note: g(x) ≤ f(x) for int x in 1:n
# and all length (l), width (w), and resolution (res) are in meters.
function generate_map(l::Real, w::Real, f::Function; g::Function=zero, res=5e-2, low=0.0,
    high=1.0, und=Inf64)
    # get map dims; initialize map to free space
    @assert (res ≤ l) && (res ≤ w)
    m, n = (w / res, l / res) .|> round .|> Int
    map = fill(low, (m, n))

    # iterate over length to assign elements with high/undefined values according to f and g
    for x = 1:n
        f_val, g_val = (f(x), g(x)) .|> round .|> Int
        f_idx = (0 ≤ f_val) ? min(f_val + 1, m) : continue  # object not present if f(x) < 0
        g_idx = (0 ≤ g_val) ? min(g_val + 1, f_idx) : 1     # no undef elements if g(x) < 0
        obj_vec = vcat(fill(und, (g_idx - 1, 1)), fill(high, (f_idx - g_idx + 1, 1)))
        @inbounds map[1:f_idx, x] = obj_vec # slice in object val vector
    end

    return Map(map, res, low, high, und)
end


"""
map plotting utils
"""

# plots map matrix as a heatmap with appropriate dimensions along the axes. Δx and Δy are
# the x and y tick step sizes for the plot, respectively. xₛ and yₛ are the starting steps
# for x and y axes so that ticks near origin do not collide. all units in are in meters.
function plot_map(map::Map; Δx=0.5, Δy=0.5, xₛ=0.0, yₛ=0.1)
    @unpack map, res = map
    m, n = size(map)
    heatmap(map)
    xticks!([(xₛ / res):(Δx / res):n; n], [xₛ:Δx:(n * res); (n * res)] .|> string)
    yticks!([(yₛ / res):(Δy / res):m; m], [yₛ:Δy:(m * res); (m * res)] .|> string)
    xlabel!("Length (m)")
    ylabel!("Width (m)")
end
