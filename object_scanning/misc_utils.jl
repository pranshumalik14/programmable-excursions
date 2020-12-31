"""
Miscellaneous utilities (types, functions, ...) to provide a basis for generating a smooth
scanning profile around an object, preferably a constant distance away from the surface
while accounting for other objects/obstacles in the way. Functions to plot map, points, ...
are also provided.
"""

using Plots
using Parameters
using LinearAlgebra
using StaticArrays

# !!todo!!: the current member type system (in frames, poses, points) restricts usage
# either make them separately inferred or just have all of them as floats

# !!todo!!: construction of a Pose2 with 𝑈 and 𝑉 having the same names should be
# automatically made Zero2; with a warning message in the logger

# !!todo!!:for default names, create a unique numbered (static global variable) frame so
# that unwanted degeneration from Pose2 to Zero2 doesn't occur

"""
Types and struct definitions for holding map and pose information
"""

abstract type GeometricEntity end
abstract type AbstractPose <: GeometricEntity end
abstract type AbstractFrame <: GeometricEntity end
abstract type AbstractPoint <: GeometricEntity end

# holds 2D map and its properties to hold the (sliced) object and environment information
@with_kw mutable struct Map{T <: Real,F <: Real}
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
@with_kw struct Frame2 <: AbstractFrame
    x::Real
    y::Real
    θ::Real # todo: conversion to (-π, π] to be looked up; also should be transform stable.
    name::AbstractString
end

# returns a world coordinate frame with origin of type T
𝑊() = Frame2(0, 0, 0, "world")

# holds 2D point information; a 2D bounded vector wrt frame {𝑉}
@with_kw mutable struct Point2 <: AbstractPoint
    x::Real
    y::Real
    𝑉::Frame2 = 𝑊()
end

# type alias for a 2D point (inferred as a bound-to-frame vector)
const Vector2 = Point2

# representation of 2D relative pose, ᵛξᵤ, head (pseudo) frame {𝑈} wrt tail frame {𝑉} or
# rigid body motion from {𝑈} to {𝑉}. the default reference frame {𝑉} is the world frame.
# the 2D pose entity lives in SE(2).
@with_kw mutable struct Pose2 <: AbstractPose
    𝑈::Frame2 = Frame2(0, 0, 0, "unnamed") # Frame{pose head}
    𝑉::Frame2 = 𝑊() # Frame{pose tail/base (reference)}
end

# explicit representation of a zero pose
struct Zero2 <: AbstractPose
    𝑈::Frame2
    𝑉::Frame2

    function Zero2()
        𝑈 = Frame2(0, 0, 0, "zero")
        𝑉 = 𝑈
        new(𝑈, 𝑉)
    end
end

# returns a zero relative pose of type T <: Real
𝛰() = Zero2()

# type alias for union of all 2D geometric entities
const GeometricEntity2D = Union{Pose2,Point2,Frame2,Zero2}

# returns a rotation transformation from 2D frame {𝑈} to 2D frame {𝑉}, ᵛRᵤ
function rot2(𝑈::Frame2, 𝑉::Frame2)
    θ = 𝑈.θ + 𝑉.θ
    return Matrix{Float64}([cos(θ) -sin(θ);
                            sin(θ)  cos(θ)])
end

# returns a translation vector from 2D frame {𝑉} to 2D frame {𝑈}, ᵛtᵤ
function transl2(𝑈::Frame2, 𝑉::Frame2)
    return [𝑈.x + 𝑉.x, 𝑈.y + 𝑉.y]
end


"""
Custom constructors and field accessors for pose and point.
"""

# Pose2(x,y,θ; name=head_frame_name, 𝑉=base_frame); todo: type constraint for all params is
# a bit annoying -- change to Real for all x, y, θ separately
function Pose2(x::Real, y::Real, θ::Real; name::S="unnamed", 𝑉::Frame2=𝑊()) where
    {S <: AbstractString}
    𝑈 = Frame2(x, y, θ, name)
    Pose2(𝑈, 𝑉)
end

# Point2(x,y)
function Point2(x::Real, y::Real)
    Point2(x, y, 𝑊())
end

# Point2(ᵛξᵤ) = Point2(ᵛ{𝑈}); creates the origin of the frame {𝑈} wrt frame {𝑉}
function Point2(ξ::P) where {P <: Union{Pose2,Zero2}}
    ξ ⋅ Point2(0.0, 0.0, ξ.𝑈)
end

# Point2(ʷ{𝐹}); origin of frame {𝐹} wrt world
function Point2(𝑈::Frame2)
    Point2(𝑈.x, 𝑈.y, 𝑊())
end

# Pose2 custom field accessors for ease of use (Pose2.{x,y,θ})
function Base.getproperty(ξ::P, field::Symbol) where {P <: Union{Pose2,Zero2}}
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
5. ξ ⊖ 𝛰 = ξ; ξ ⊕ 𝛰 = ξ
6. ᵗξᵤ ⋅ ᵘp = ᵗp
"""

# oplus operator for pose; by default requires base and reference frames to be same
function ⊕(ξ₁::Union{Pose2,Zero2}, ξ₂::Union{Pose2,Zero2})
    if ξ₁ isa Zero2
        return ξ₂
    elseif ξ₂ isa Zero2
        return ξ₁
    else
        @assert ξ₁.𝑈.name == ξ₂.𝑉.name
        if (ξ₁.𝑉.name == ξ₂.𝑈.name) return 𝛰() end
        x, y, θ = compose2(@SVector([ξ₁.x, ξ₁.y, ξ₁.θ]), @SVector([ξ₂.x, ξ₂.y, ξ₂.θ]))
        𝑈 = Frame2(x, y, θ, ξ₂.name)
        return Pose2(𝑈, ξ₁.𝑉) # todo: check if this is same as Pose2(x₁ + x₂, y₁ + y₂, θ₁ + θ₂)
    end
end

# compose operator for pose, with no frame assertion; <returns ξ₁ ⊕ ξ₂ wrt {ξ₁.𝑉} ???>
function Base.:∘(ξ₁::Union{Pose2,Zero2}, ξ₂::Union{Pose2,Zero2})
    if ξ₁ isa Zero2
        return ξ₂
    elseif ξ₂ isa Zero2
        return ξ₁
    else
        x, y, θ = compose2(@SVector([ξ₁.x, ξ₁.y, ξ₁.θ]), @SVector([ξ₂.x, ξ₂.y, ξ₂.θ]))
        𝑈 = Frame2(x, y, θ, ξ₂.name)
        return Pose2(𝑈, ξ₁.𝑉) # todo: check if this is same as Pose2(x₁ + x₂, y₁ + y₂, θ₁ + θ₂)
    end
end

# returns (x, y, θ) ∼ T1 ∘ T2 where T ∈ SE(2)
function compose2(T₁::SVector{3}, T₂::SVector{3})
    𝑅₁ = @SMatrix   [cos(T₁[3]) -sin(T₁[3]);
                     sin(T₁[3])  cos(T₁[3]);]
    𝑇₁ = [[𝑅₁ T₁[1:2]]; SA[0 0 1]]

    𝑅₂ = @SMatrix   [cos(T₂[3]) -sin(T₂[3]);
                     sin(T₂[3])  cos(T₂[3]);]
    𝑇₂ = [[𝑅₂ T₂[1:2]]; SA[0 0 1]]

    𝑇 = 𝑇₁ * 𝑇₂

    return (𝑇[1,3], 𝑇[2, 3], atan(𝑇[2, 1], 𝑇[1, 1])) # x, y, θ of the composed transform
end

# plus binary operator for pose, with no frame assertion; <returns ξ₁ ⊕ ξ₂ wrt {ξ₁.𝑉} ???>
Base.:+(ξ₁::Union{Pose2,Zero2}, ξ₂::Union{Pose2,Zero2}) = Base.:∘(ξ₁, ξ₂)

# minus unary operator for pose
function Base.:-(ξ::P) where {P <: Union{Pose2,Zero2}}
    # get global frame ʷ{𝑈} and then return inverse relative pose
    𝑇 = @SMatrix   [cos(ξ.θ) -sin(ξ.θ) ξ.x;
                    sin(ξ.θ)  cos(ξ.θ) ξ.y;
                    0         0          1]
    x, y, θ = compose2(@SVector([ξ.𝑉.x, ξ.𝑉.y, ξ.𝑉.θ]), @SVector([ξ.x, ξ.y, ξ.θ]))
    ʷ𝑈 = Frame2(x, y, θ, ξ.name)
    𝑇⁻¹ = inv(𝑇) # todo: check if ∼ (-x,-y, -θ)

    return Pose2(𝑇⁻¹[1,3], 𝑇⁻¹[2, 3], atan(𝑇⁻¹[2, 1], 𝑇⁻¹[1, 1]); name=ξ.𝑉.name,
        𝑉=ʷ𝑈)
end

# ominus binary operator for pose
⊖(ξ₁::Union{Pose2,Zero2}, ξ₂::Union{Pose2,Zero2}) = ξ₁ ⊕ -(ξ₂)

# minus binary operator for pose; composes ξ₁ and -(ξ₂) without frame assertion
Base.:-(ξ₁::Union{Pose2,Zero2}, ξ₂::Union{Pose2,Zero2}) = ξ₁ ∘ -(ξ₂)

# dot operator for point frame transformation by a relative pose, ᵛξᵤ ⋅ ᵘp = ᵛp
function ⋅(ξ::P, p::Point2) where {P <: Union{Pose2,Zero2}}
    if ξ isa Zero2 && p.𝑉.name ∈ ("world", "zero")
        return p
    end

    # point should be relative to head frame of the pose
    @assert ξ.𝑈.name == p.𝑉.name && p.𝑉.name ≠ "null"

    # 2D homogenous transform from {𝑈} to {𝑉}
    ᵛ𝑅ᵤ = rot2(ξ.𝑈, ξ.𝑉)
    ᵛ𝑡ᵤ = transl2(ξ.𝑈, ξ.𝑉)
    ᵛ𝑇ᵤ = Matrix{Float64}([ᵛ𝑅ᵤ  ᵛ𝑡ᵤ;
                            0  0  1])
    ᵘp̃ = [p.x, p.y, 1]      # homogenous vector for source point
    ᵛx, ᵛy, _ = ᵛ𝑇ᵤ * ᵘp̃    # homogenous vector for target point

    return Point2(ᵛx, ᵛy, ξ.𝑉)
end

# extending `+` binary operator for 2D points
function Base.:+(p₁::Point2, p₂::Point2)
    # only operate on points/vectors in the same reference frame
    @assert p₁.𝑉.name ≠ "null" && p₂.𝑉.name ≠ "null"
    @assert p₁.𝑉.name == p₂.𝑉.name
    return Point2(p₁.x + p₂.x, p₁.y + p₂.y, p₁.𝑉)
end

# extending `-` binary operator for 2D points
function Base.:-(p₁::Point2, p₂::Point2)
    # only operate on points/vectors in the same reference frame; output wrt p₂ (hence null)
    @assert p₁.𝑉.name ≠ "null" && p₂.𝑉.name ≠ "null"
    @assert p₁.𝑉.name == p₂.𝑉.name
    return Point2(p₁.x - p₂.x, p₁.y - p₂.y, Frame2(0, 0, 0, "null"))
end

# extending `-` unary operator for 2D points
Base.:-(p::Point2) = Point2(-p.x, - p.y, p.𝑉)

# returns the Lᵖ-norm of a 2D pose or point. extending the base linear algebra method.
function LinearAlgebra.norm(p2::GeometricEntity2D; p::Real=2)
    return LinearAlgebra.norm([p2.x, p2.y], p)
end


"""
Map setup utils
"""

# returns a matrix of size w/res × l/res, filling elements between and including the two
# scalar functions f and g with the high value. rest of the map above f is filled with low
# value and below g is filled with the undefined value. note: g(x) ≤ f(x) for int x in 1:n
# and the arguments length (l), width (w), and resolution (res) are in meters.
function generate_map(l::Real, w::Real, f::Function; g::Function=zero, res=1e-3, low=0.0,
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

# plots a 2D geometric entity on map scale
function plot_point(g::G₂, map::Map) where {G₂ <: GeometricEntity2D}
    @unpack res = map
    scatter!([(g.x / res)], [(g.y / res)], legend=false)
end
