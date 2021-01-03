### A Pluto.jl notebook ###
# v0.12.18

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : missing
        el
    end
end

# ╔═╡ 3bda4224-4970-11eb-29ac-091d674c6763
begin
	include("misc_utils.jl");
	using PlutoUI;
end

# ╔═╡ 50f53aaa-49f5-11eb-0d5a-cb3e9c45649a
md"

# Pose algebra

We can operate on poses and points by the following rules:
- Pose addition/composition: ${^T\mathbf{\xi}_U} \oplus {^U\mathbf{\xi}_V} = {^T\mathbf{\xi}_V}$
- Non-commutative composition, order matters: $\mathbf{\xi}_1 \oplus \mathbf{\xi}_2 \neq \mathbf{\xi}_2 \oplus \mathbf{\xi}_1$
- Additive inverse of a pose: $\ominus {^T\mathbf{\xi}_U} = {^U\mathbf{\xi}_T}$
- Zero pose: $\mathbf{\xi} \ominus \mathbf{\xi} = \mathbf{0}$ and $\ominus \mathbf{\xi} \oplus \mathbf{\xi} = \mathbf{0}$
- Additive identity of a pose: $\mathbf{\xi} \oplus \mathbf{0} = \mathbf{\xi}$ and $\mathbf{\xi} \oplus \mathbf{0} = \mathbf{\xi}$
- Transformation of a point from one frame of reference to another, by a relative pose: ${^T\mathbf{\xi}_U} \cdot {^U\mathbf{p}}  = {^T\mathbf{p}}$

Note: 
- Use `-` instead of `⊖` for unary $\ominus$ operator in code since Julia does not allow definition of custom unary operators because of parsing constraints.
- Use `∘` or `+` operator in code to compose two poses **without** frame assertion. These operators are *blind* to the reference and resultant frames of the pose on rhs, $\mathbf{\tilde\xi}$ in ${^X\mathbf{\xi}_Y} \circ {^{-}\mathbf{\tilde\xi}_{-}} = {^X\mathbf{\xi}'}$.
- Use `-` binary operator in code to compose with inverse of the rhs pose **without** frame assertion.
- Use `⊕` and `⊖`  binary operators in code to enforce frame assertion according to the rules above.
- Frame {`𝑉`} in `Pose2`, and `Point2` specifies the reference frame pose with respect to the world origin, `𝑊() = Frame2(0, 0, 0, \"world\")`.
- Frame {`𝑈`} in `Pose2` specifies the relative pose, ${^V\mathbf{\xi}_U} \in \text{SE}(2)$, of {`𝑈`} with respect to {`𝑉`}. In code, ${^V\mathbf{\xi}_U} \sim {^V(x, y, \theta)_U}$

"

# ╔═╡ 701ff10c-4a1d-11eb-0b65-59b45daa23c2
# basic usage example

begin
	ʷξₗ = Pose2(1, 2, π/6; name="base_link")   # base link at (1,2) @ 30°
	𝐿   = Frame2(ʷξₗ)							# global frame for the base link
	ˡξₕ = Pose2(2, 1, 0; name="head", 𝑉=𝐿)	 # head at (2,1) wrt base link
	𝐻 	 = Frame2(ˡξₕ)							# global frame for robot head
	ˡp  = Point2(3, 3, 𝐿)						# obst at (3,3) wrt base link
	ʰp  = (- ˡξₕ) ⋅ ˡp 						   # obst location wrt head
	ʷp  = ʷξₗ ⋅ ˡp							   # obst location wrt world
	@assert ʷp ≈ (ʷξₗ ⊕ ˡξₕ) ⋅ ʰp 			   # check if composition returns same pt.
	
	ʷξₗ = ʷξₗ ⊕ Pose2(0, 0, π/3; name=ʷξₗ.name, 𝑉=𝐿) # rotate base link by 60°
	𝐿̂  = Frame2(ʷξₗ); 𝐻̂ = Frame2(ˡξₕ);			     # updated base and head frames
	ˡp̃  = (- ʷξₗ) ⋅ ʷp 			# get old point in new frame through world frame
	ʰp̃  = (- ˡξₕ) ⋅ ˡp̃ 			 # get point in new head frame
	ʷp̃  = ʷξₗ ⋅ ˡp̃ 				 # get point back in world frame from new base frame
	@assert ʷp̃ ≈ (ʷξₗ ⊕ ˡξₕ) ⋅ ʰp̃  # check for correctness of composition
	
	# show final results wrt world
	with_terminal() do
		@show ˡp; @show ʰp; @show ˡp̃; @show ʰp̃; @show ʷp;
	end
end

# ╔═╡ e6de0e36-496f-11eb-32e9-7f92c36296a3
md"

# Heuristic evaluation utils
To scan a 2D slice of an object, we first need to define a desired padding distance from the surface, 𝑑ₚ. In order to inform the path planner about this criterion, a heuristic needs to be constructed. A simple quadratic penalty method, penalizing a point based on its norm from the padded curve, can help the final plan converge to the desired scanning profile.

"

# ╔═╡ 7307a8e6-4dde-11eb-26cb-6bdd3881b940
md"

## Piecewise Linear Approximation
We can divide an arbitrary, curved object into sections and treat each of them as 

Length, `ℓ` = 
$(@bind ℓ Slider(0.5:0.01:2.5; default=1.5, show_value=true))

Width, `𝑤` = 
$(@bind 𝑤 Slider(0.2:0.01:1.5; default=0.75, show_value=true))

Scan padding distance, `𝑑ₚ` =
$(@bind 𝑑ₚ Slider(0.25:0.01:√2𝑤; default=round(3*√2𝑤/4; digits=2), show_value=true))

surface = x -> 2 * sin(0.05x) + 0.12x
bottom = ;
A slider for their slopes, so that I can offset and move the aisle around.

Plot map and the mod points at Δpose.

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
generate_map(ℓ, 𝑤, surface; g=bottom) |> plot_map

# ╔═╡ Cell order:
# ╟─3bda4224-4970-11eb-29ac-091d674c6763
# ╟─50f53aaa-49f5-11eb-0d5a-cb3e9c45649a
# ╠═701ff10c-4a1d-11eb-0b65-59b45daa23c2
# ╟─e6de0e36-496f-11eb-32e9-7f92c36296a3
# ╠═7307a8e6-4dde-11eb-26cb-6bdd3881b940
# ╠═a3f57948-4970-11eb-25f0-0d3d60aa888e
# ╠═9d2b16ce-4971-11eb-0172-997483dda6fb
