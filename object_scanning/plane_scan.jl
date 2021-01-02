### A Pluto.jl notebook ###
# v0.12.18

using Markdown
using InteractiveUtils

# ╔═╡ 3bda4224-4970-11eb-29ac-091d674c6763
begin
	include("misc_utils.jl");
	using PlutoUI;
end

# ╔═╡ 50f53aaa-49f5-11eb-0d5a-cb3e9c45649a
md"

# Pose algebra

We can operate on poses and points by the following rules:
- Pose addition: ${^T\mathbf{\xi}_U} \oplus {^U\mathbf{\xi}_V} = {^T\mathbf{\xi}_V}$
- No commutativity, order matters: $\mathbf{\xi}_1 \oplus \mathbf{\xi}_2 \neq \mathbf{\xi}_2 \oplus \mathbf{\xi}_1$
- Additive inverse of a pose: $\ominus {^T\mathbf{\xi}_U} = {^U\mathbf{\xi}_T}$
- Zero pose definition: $\mathbf{\xi} \ominus \mathbf{\xi} = \mathbf{0}$ and $\ominus \mathbf{\xi} \oplus \mathbf{\xi} = \mathbf{0}$
- Additive identity of a pose: $\mathbf{\xi} \oplus \mathbf{0} = \mathbf{\xi}$ and $\mathbf{\xi} \oplus \mathbf{0} = \mathbf{\xi}$
- Transformation of a point by a relative pose: ${^T\mathbf{\xi}_U} \cdot {^U\mathbf{p}}  = {^T\mathbf{p}}$

Note: 
- Use `-` instead of `⊖` for unary $\ominus$ operator in code since Julia does not allow definition of custom unary operators because of parsing constraints.
- Use `∘` or `+` operator in code to compose two poses **without** frame assertion. These operators are *blind* to the reference frame of the rhs pose, $\tilde\xi_0$ in ${^X\xi_Y} \circ {^{-}\tilde\xi_o}$.
- Use `-` binary operator in code to compose with inverse of a pose **without** frame assertion.
- Use `⊖` and `⊕` binary operators in code to enforce frame assertion according to the rules above.
"

# ╔═╡ 701ff10c-4a1d-11eb-0b65-59b45daa23c2
begin
	ʷξₒ = Pose2(0.0, 0.0, 0.0, name="origin")
	ᵒξᵤ = Pose2(1.0, 2.0, 0.0, name="head", 𝑉=ʷξₒ.𝑈)
	ᵘp = Point2(ᵒξᵤ)
	x = ʷξₒ ⋅ ᵘp
	
	with_terminal() do 
		@show ʷξₒ; @show ᵒξᵤ; @show ᵘp; @show x 
	end
end

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
generate_map(1.5, 0.45, x -> 2 * sin(0.05x) + 0.12x) |> plot_map

# ╔═╡ 4184fefa-49f5-11eb-336b-4d9fe1466530
begin
	ᵒξᵤ.𝑈
end

# ╔═╡ Cell order:
# ╟─3bda4224-4970-11eb-29ac-091d674c6763
# ╟─50f53aaa-49f5-11eb-0d5a-cb3e9c45649a
# ╠═701ff10c-4a1d-11eb-0b65-59b45daa23c2
# ╟─e6de0e36-496f-11eb-32e9-7f92c36296a3
# ╠═a3f57948-4970-11eb-25f0-0d3d60aa888e
# ╠═9d2b16ce-4971-11eb-0172-997483dda6fb
# ╠═4184fefa-49f5-11eb-336b-4d9fe1466530
