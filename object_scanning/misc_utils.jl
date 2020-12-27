"""
Miscellaneous functions to facilitate generation and plotting of a smooth scanning profile
around the object, preferably a constant distance away from the surface and accounting for
other objects/obstacles in the way.
"""

using Plots
using Parameters
using LinearAlgebra


"""
Types and struct definitions for holding map and pose information
"""

abstract type GeometricEntity end
abstract type AbstractPose <: GeometricEntity end
abstract type AbstractFrame <: GeometricEntity end
abstract type AbstractPoint <: GeometricEntity end

# holds 2D map and its properties to hold the (sliced) object and environment information
@with_kw mutable struct Map{T <: Real,F <: AbstractFloat}
    map::Matrix{T}
    low::T
    high::T
    und::T
    res::F
end

# represents a 2D frame, (implicitly) wrt world frame. a frame is immutable, and by
# default, defined wrt the (right-handed) world  coordinate frame.  note that, by
# convention, θ (rad) increases in the anticlockwise direction. the 2D frame entity lives
# in SE(2).
@with_kw struct Frame2{T <: Real,S <: AbstractString} <: AbstractFrame
    x::T
    y::T
    θ::T
    name::S
end

# returns a world coordinate frame with origin of type T
𝑊(T) = Frame2{T,String}(0, 0, 0, "World")

# holds 2D point information; a 2D bounded vector wrt frame {𝑈}
@with_kw mutable struct Point2{T <: Real} <: AbstractPoint
    x::T
    y::T
    𝑈::Frame2{T,<:AbstractString} = 𝑊(T)
end

# type alias for a 2D point (inferred as a bound-to-frame vector)
const Vector2 = Point2

# representation of 2D relative pose, ᵛξᵤ, head frame {𝑈} wrt tail frame {𝑉} or rigid body
# motion from {𝑈} to {𝑉}. the default reference frame {𝑉} is the world frame. the 2D
# pose entity lives in SE(2).
@with_kw mutable struct Pose2{T <: Real} <: AbstractPose
    𝑈::Frame2{T,<:AbstractString} = Frame2{T,String}(0, 0, 0, "unnamed") # Frame{pose head}
    𝑉::Frame2{T,<:AbstractString} = 𝑊(T) # Frame{pose tail/base (reference)}
end

# explicit representation of a zero pose
@with_kw struct Zero2{T <: Real} <: AbstractPose
    𝑈::Frame2{T,<:AbstractString} = Frame2{T,String}(0, 0, 0, "zero")
    𝑉::Frame2{T,<:AbstractString} = 𝑈
end

# returns a zero relative pose of type T
𝛰(T) = Zero2{T}()

# type alias for union of all 2D geometric entities
const GeometricEntity2D = Union{Pose2,Point2,Frame2,Zero2}


"""
Custom constructors and field accessors for pose and point.
"""

# Pose2(x,y,θ; name=head_frame_name, 𝑉=base_frame)
function Pose2(x::T, y::T, θ::T; name::S="unnamed", 𝑉::AbstractFrame=𝑊(T)) where
    {T <: Real,S <: AbstractString}
    𝑈 = Frame2(x, y, θ, name)
    Pose2(𝑈, 𝑉)
end

# Point2(x,y)
function Point2(x::T, y::T) where {T <: Real}
    Point2(x, y, 𝑊(T))
end

# Point2(ᵛξᵤ)
function Point2(ξ::P) where {P <: AbstractPose}
    Point2(ξ.x, ξ.y, ξ.𝑉)
end

# Pose2 custom field accessors for ease of use (Pose2.{x,y,θ})
function getproperty(ξ::P, field::Symbol) where {P <: AbstractPose}
    if field ∈ (:𝑈, :𝑉)     # head/base frames
        return getfield(ξ, field)
    elseif field === :x     # x coordinate
        return getfield(getfield(ξ, :𝑈), :x)
    elseif field === :y     # y coordinate
        return getfield(getfield(ξ, :𝑈), :y)
    elseif field === :θ     # angle θ (ccw); head frame {𝑈} to relative to base frame {𝑉}
        return getfield(getfield(ξ, :𝑈), :θ)
    elseif field === :name  # name of the head frame
        return getfield(getfield(ξ, :𝑈), :name)
    else
        error("Property $field for $ξ not defined!")
    end
end


"""
Pose and point operations and algebra:
1. ᵗξᵤ ⊕ ᵘξᵥ = ᵗξᵥ
2. ξ₁ ⊕ ξ₂ ≠ ξ₂ ⊕ ξ₁
3. ⊖ ᵗξᵤ = ᵘξₜ
4. ξ ⊖ ξ = 𝛰; ⊖ ξ ⊕ ξ = 𝛰
5. ξ ⊖ 𝛰 = ξ; ⊖ ξ ⊕ 𝛰 = 𝛰
6. ᵗξᵤ ⋅ ᵘp = ᵗp
"""

# oplus operator for pose
function ⊕(ξ₁::AbstractPose, ξ₂::AbstractPose)
    if ξ₁ isa Zero2
        return ξ₂
    elseif ξ₂ isa Zero2
        return ξ₁
    else
        @assert (ξ₁.𝑈 == ξ₂.𝑉)
        return (ξ₁.𝑉 == ξ₂.𝑈) ? Pose2(ξ₂.𝑈, ξ₁.𝑉) : 𝛰(typeof(ξ₁.x))
    end
end

# ominus unary operator for pose
function -(ξ::P) where {P <: AbstractPose}
    return Pose2(ξ.𝑉, ξ.𝑈) # flip reference and head frames
end

# ominus unary operator for pose
function -(ξ₁::AbstractPose, ξ₂::AbstractPose)
    return ξ₁ ⊕ -(ξ₂)
end

function ⋅(ξ::Pₛ, p::Pₜ) where {Pₛ <: AbstractPose,Pₜ <: AbstractPoint}
    #
end

function +(p₁::Point2, p₂::Point2)
    @assert p₁.𝑈 == p₂.𝑈
    return Point2(p₁.x + p₂.x, p₁.y + p₂.y, p₁.𝑈)
end

function -(p₁::P, p₂::P) where {P <: AbstractPoint}
    @assert p₁.𝑈 == p₂.𝑈
    return Point2(p₁.x - p₂.x, p₁.y - p₂.y, p₁.𝑈)
end

# returns the Lᵖ-norm of a 2D pose or point. extending the base linear algebra method.
function LinearAlgebra.norm(p2::GeometricEntity2D; p::Real=2)
    return LinearAlgebra.norm([p2.x, p2.y], p)
end


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
Map setup utils
"""

# returns a matrix of size w/res × l/res, filling elements between and including the two
# scalar functions f and g with the high value. rest of the map above f is filled with low
# value and below g is filled with the undefined value. note: g(x) ≤ f(x) for int x in 1:n
# and the arguments length (l), width (w), and resolution (res) are in meters.
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

    return Map(map, low, high, und, res)
end


"""
Map and pose plotting utils
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

function plot_pose()
    #
end

function plot_obj_ref_frames()
    #
end

function plot_scan_ref_frames()
    #
end

function plot_scan_heuristic(map::Map)
    plot_map(map)
    plot_obj_ref_frames()   # optional
    plot_scan_ref_frames()  # optional
end
