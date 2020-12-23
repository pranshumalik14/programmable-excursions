"""
Miscellaneous functions to facilitate generation and plotting of a smooth scanning profile
around the object, preferably a constant distance away from the surface and accounting for
other objects/obstacles in the way.
"""

using Plots
using Parameters


"""
Struct definitions for holding map and pose information
"""

# holds 2D map and its properties to hold the (sliced) object and environment information
mutable struct Map
    map::Matrix{Real}
    res::Real
    low::Real
    high::Real
    und::Real
end

# holds frame information (immutable)
struct Frame
    x::Real
    y::Real
    θ::Real
    name::AbstractString
end

# holds 2D point information; a vector
@with_kw mutable struct Point2
    x::Real
    y::Real
    𝓊::Frame
end

# holds 2D pose information; note that, by convention, θ increases in the anticlockwise
# direction; pose is stored for frame 𝓊 with respect to frame 𝓋.
@with_kw mutable struct Pose2
    x::Real
    y::Real
    θ::Real
    𝓊::Frame
    𝓋::Frame
end


"""
Pose operations
"""

# translate (SE2)
# rotate (SE2)
# frame change, pose diff, pose add, pose struct
# ᵛp = ᵛξᵤ ⋅ ᵘp -> dot operator for Pose to point. Will assert base frames to be the same
# ᵛξₜ = ᵛξᵤ ⊕ ᵘξₜ -> oplus operator for pose to pose. Will assert base frames to be the same


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
function generate_map(l::Real, w::Real, f::Function; g::Function=zero, res=5e-2, low=0.0,
    high=1.0, und=Inf64)
    # get map dims; initialize map to free space
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

    return map
end


"""
map plotting utils
"""

# returns
function plot_map(map::Matrix{<:Real})
    return heatmap(map)
end

f = x -> 0.5 * x - sin(0.5x) * x * 0.1
g = x -> 0.25 * x - x * cos(0.25x) * 0.1
generate_map(5.5, 2, f; g=g) |> plot_map
