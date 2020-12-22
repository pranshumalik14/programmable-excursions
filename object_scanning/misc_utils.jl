"""
Miscellaneous functions to facilitate generation and plotting of a smooth scanning profile
around the object, preferably a constant distance away from the surface and accounting for
other objects/obstacles in the way.
"""

using Plots


"""
map setup utils
"""

# returns a matrix of size w/res × l/res, filling elements below and including int output
# of the scalar function with the high value. rest of the matrix is filled with low value
function generate_map(f::Function, l::Real, w::Real; res=5e-2, low=0.0, high=1.0)
    m, n = (w / res, l / res) .|> round .|> Int
    map = fill(low, (m, n))

    # iterate over length to assign elements with high value, up till the function value
    for x = 1:n
        f_val = f(x) |> round |> Int
        height_idx = (0 ≤ f_val ≤ m - 1) ? f_val + 1 : continue
        @inbounds map[1:height_idx, x] = fill(high, (height_idx, 1)) # slice in high val vec
    end

    return map
end


"""
map plotting utils
"""

# returns
function plot_map(map::Matrix{<:Number})
    return heatmap(map)
end
