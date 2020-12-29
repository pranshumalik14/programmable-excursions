### A Pluto.jl notebook ###
# v0.12.17

using Markdown
using InteractiveUtils

# ╔═╡ 3bda4224-4970-11eb-29ac-091d674c6763
include("misc_utils.jl");

# ╔═╡ e6de0e36-496f-11eb-32e9-7f92c36296a3
md"

# Heuristic evaluation utils
To scan a 2D slice of an object, we first need to define a desired buffer distance from the surface. In order to stay at a distance, a special heuristic needs to be fed into the planner to inform it about this criterion. For this, a simple penalty method can help the final plan converge to the desired distance.

"

# ╔═╡ a3f57948-4970-11eb-25f0-0d3d60aa888e
# return heuristic per object point
# return heuristic points for collection
# heuristic evaluation over map

# mutates free space elements (low val)
function evaluate_heuristic()
    #
end

# ╔═╡ 9d2b16ce-4971-11eb-0172-997483dda6fb
generate_map(1.5, 0.45, x -> 2*sin(0.05x) + 0.12x) |> plot_map

# ╔═╡ Cell order:
# ╠═3bda4224-4970-11eb-29ac-091d674c6763
# ╟─e6de0e36-496f-11eb-32e9-7f92c36296a3
# ╠═a3f57948-4970-11eb-25f0-0d3d60aa888e
# ╠═9d2b16ce-4971-11eb-0172-997483dda6fb
