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
	using Plots;
	using Statistics;
	using LinearAlgebra;
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

let
	ʷξₗ = Pose2(1, 2, π/6; name="base_link")   # base link at (1,2) @ 30°
	𝐿   = Frame2(ʷξₗ)							# global frame for the base link
	ˡξₕ = Pose2(2, 1, 0; name="head", 𝑉=𝐿)	 # head at (2,1) wrt base link
	𝐻 	 = Frame2(ˡξₕ)							# global frame for robot head
	ˡp  = Point2(3, 3, 𝐿)						# obst at (3,3) wrt base link
	ʰp  = (- ˡξₕ) ⋅ ˡp 						   # obst location wrt head
	ʷp  = ʷξₗ ⋅ ˡp							   # obst location wrt world
	@assert ʷp ≈ (ʷξₗ ⊕ ˡξₕ) ⋅ ʰp 			   # check if composition returns same pt.

	ʷξₗ = ʷξₗ ⊕ Pose2(0, 0, π/3; name=ʷξₗ.name, 𝑉=𝐿) # rotate base link by 60°
	𝐿̂  = Frame2(ʷξₗ); ˡξₕ.𝑉 = 𝐿̂; 𝐻̂ = Frame2(ˡξₕ);	# updated base and head frames
	ˡp̃  = (- ʷξₗ) ⋅ ʷp 			# get old point in new frame through world frame
	ʰp̃  = (- ˡξₕ) ⋅ ˡp̃ 			 # get point in new head frame
	ʷp̃  = ʷξₗ ⋅ ˡp̃ 				 # get point back in world frame from new base frame
	@assert ʷp̃ ≈ (ʷξₗ ⊕ ˡξₕ) ⋅ ʰp̃  # check for correctness of composition
	
	# check if point remained the same wrt all reference frames
	@assert ˡp ≈ ʰp ≈ ʷp ≈ ʷp̃ ≈ ʰp̃ ≈ ˡp̃

	# show final results wrt world
	with_terminal() do
		@show ˡp; @show ʰp; @show ˡp̃; @show ʰp̃; @show ʷp;
	end
end

# ╔═╡ e6de0e36-496f-11eb-32e9-7f92c36296a3
md"

# Heuristic evaluation
To scan a 2D slice of an object, we first need to define a desired padding distance, 𝑑ₚ, from the surface. Based on this distance, a heuristic can be constructed to inform the path planner about this criterion. A simple quadratic penalty method, penalizing a point based on its norm from the padded curve, can help the final plan converge to the desired scanning profile. Note that the scanning profile is going to be different from the padding profile, which does not take obstacles and kinodynamic constraints into account.

"

# ╔═╡ 7307a8e6-4dde-11eb-26cb-6bdd3881b940
md"

## Piecewise Linear Approximation
We can divide an arbitrary, curved object into linear sections and run the penalty heuristic separately on all sections to get the complete padding profile. To define such a section, we can specify the length and width for the region we want to investigate along with the padding distance and exterior and interior surface functions defining the object in this region.

Length, `ℓ` =
$(@bind ℓ Slider(0.5:0.01:2.5; default=1.5, show_value=true))

Width, `𝑤` =
$(@bind 𝑤 Slider(0.2:0.01:1.5; default=0.5, show_value=true))

Scan padding distance, `𝑑ₚ` =
$(@bind 𝑑ₚ Slider(0.25:0.01:√2𝑤; default=round(3*√2𝑤/4; digits=2), show_value=true))

Exterior surface, $f_e(x) = ax^2 + bx + c$

𝑎 =
$(@bind a Slider(1:0.01:30; default=18.75, show_value=true)),
𝑏 =
$(@bind b Slider(0.0:0.01:2.0; default=0.12, show_value=true)),
𝑐 =
$(@bind c Slider(-𝑤:0.01:𝑤; default=0.03, show_value=true))

Interior surface, $f_i(x) = \tilde{a}x^2 + \tilde{b}x + \tilde{c}$

𝑎̃ =
$(@bind ã Slider(0:0.01:10; default=0, show_value=true)),
𝑏̃ =
$(@bind b̃ Slider(0.0:0.01:1.2; default=b, show_value=true)),
𝑐̃ =
$(@bind c̃ Slider(-𝑤:0.01:𝑤; default=0, show_value=true))

To efficiently estimate object exterior surface for the heuristic and avoid storing for every point, we can interpolate interior points at regular intervals, $\Delta \mathbf{p}$, between the object's start and end points. The start and end points within the selected region can be interpreted to be the mean of interior and exterior surfaces at the two extreme lengths. Note that these points can also be given externally; we are averaging here so that no extra data is required from the user for this demo.

Interpolation interval, $||\Delta \mathbf{p}||$ =
$(@bind Δp Slider(0.01:0.01:ℓ/8; default=0.05, show_value=true))

"

# ╔═╡ 754bbfc0-4ded-11eb-3719-d16818482c28
begin
	# create environment map
	res = 1e-3; high = 100.0; low = 0.0; und = Inf64 # map parameters
	fₑ = x -> a * res * x^2 + b * res * x + (c / res) 			 # exterior surface function
	fᵢ = x -> ã * res* x^2 + b̃ * x + (c̃ / res) 			# interior surface function
	env_map = generate_map(ℓ, 𝑤, fₑ; g=fᵢ, res=res, high=high, low=low, und=und)

	# start/end pts: if fᵢ ≥ fₑ then take extreme point to be ||Δp|| units below fₑ
	xₛ, xₑ = 1, ℓ / res
	yₛ = fₑ(xₛ) > fᵢ(xₛ) ? mean([fₑ(xₛ) fᵢ(xₛ)]) : fₑ(xₛ) - (Δp / res)
	yₑ = fₑ(xₑ) > fᵢ(xₑ) ? mean([fₑ(xₑ) fᵢ(xₑ)]) : fₑ(xₑ) - (Δp / res)

	# object section points in map coordinates
	θₒ      = atan(yₑ - yₛ, xₑ - xₛ) 					# θ = tan⁻¹(Δy/Δx); obj angle
	Δx, Δy 	= (Δp * cos(θₒ)) / res, (Δp * sin(θₒ)) / res# Δx and Δy in map coordinates
	x_pts, y_pts = [xₛ:Δx:xₑ;], [yₛ:Δy:yₑ;]				# unfiltered obj x, y coords
	ᵐobj_pts = [Point2(x, y) for (x, y) ∈ zip(x_pts, y_pts) 
			if checkbounds(Bool, env_map.map, y, x)]	# obj points in map coords

	# object section frame {𝑂} and object points wrt {𝑂}
	p₁ = ᵐobj_pts[1]								# first object point wrt map
	ᵐξₒ= Pose2(p₁.x, p₁.y, θₒ; name="object_frame")	# map to {𝑂} relative pose
	𝑂 	= Frame2(ᵐξₒ)							 	 # object frame (wrt map)
	ᵒobj_pts = [(- ᵐξₒ) ⋅ ᵐp for ᵐp ∈ ᵐobj_pts]		# obj points wrt {𝑂}
	ᵒΔp      = Vector2(Δp / res, 0, 𝑂) 			 # interp interval vector in {𝑂}

	# plot
	plot_map(env_map)
	plot_points(ᵐobj_pts)
end

# ╔═╡ 51175c8e-4df1-11eb-057e-4741292fbe95
md"

### Boundary approximation
To get an approximate padding profile, we first need to get the boundary of the aisle. Although, we have, in real life we would not have an object defined as a function, but we can expect a map of the environment with obstacle information. To efficiently check, we do it in increments of some check distance.

Boundary check distance, `𝑑ₖ` =
$(@bind 𝑑ₖ Slider(0.001:0.001:Δp; default=0.01, show_value=true))

We can then keep adding the normal vector scaled by 𝑑ₖ to get to the boundary. Let this vector be called the check vector, $\mathbf{\Delta c}$.

"

# ╔═╡ 98d7802a-4e65-11eb-2e34-abd3d757a2be
ᵒΔc = Vector2(0, 𝑑ₖ / res, 𝑂) # check vector in {𝑂}; normal to ᵒΔp

# ╔═╡ aeadbe7e-4e35-11eb-1f95-67a0b327f61a
# returns vector of relative poses for boundary pts, given the object pts wrt {𝑂}
function get_boundary_poses(ᵒobj_pts::Vector{Point2}, map::Map)
	boundary_poses = Vector{Pose2}()

	# translate object pts by check vector to reach boundary pts
	for ᵒpᵢ ∈ ᵒobj_pts
		ᵐpᵢ = ᵐξₒ ⋅ ᵒpᵢ

		while map_value(ᵐpᵢ.x, ᵐpᵢ.y, map) ≠ map.low
			ᵒpᵢ += ᵒΔc 		# increment object point
			ᵐpᵢ = ᵐξₒ ⋅ ᵒpᵢ # update map point
		end

		# if last step resulted in going out of map, then take one step back
		if !checkbounds(Bool, map.map, ᵐpᵢ.y, ᵐpᵢ.x)
			ᵒpᵢ += (- ᵒΔc)
		end
		
		push!(boundary_poses, Pose2(ᵒpᵢ.x, ᵒpᵢ.y, 0; 𝑉=𝑂)) # transl component only
	end

	# rotate boundary pose to orient itself towards next boundary pose (capturing
	# local curve in object boundary)
	for i ∈ 1:(length(boundary_poses) - 1)
		ᵒξᵢ, ᵒξᵢ₊₁ = boundary_poses[i:(i + 1)]
		θᵢ = atan(ᵒξᵢ₊₁.y - ᵒξᵢ.y, ᵒξᵢ₊₁.x - ᵒξᵢ.x)
		boundary_poses[i] = ᵒξᵢ ∘ Pose2(0, 0, θᵢ)
	end

	return boundary_poses
end

# ╔═╡ aa19d822-4e65-11eb-0a43-05ff6cd19730
begin
	ᵒbps = get_boundary_poses(ᵒobj_pts, env_map)
	plot_points([ᵐξₒ ⋅ Point2(ξ) for ξ ∈ ᵒbps]; color="green")
end

# ╔═╡ db8bf968-4e2f-11eb-192b-8716d8109f4c
md"

## Wrapping up: heuristic lookup
Finally to get all the points, we do this:
1. Project point onto the unit vector of the object's section.
2. Get closest point by getting closest computed frame's x or y from xₛ + NΔx. should also checkout with yₛ + NΔy
3. Once we have this frame, then get frame's (cache this
"

# ╔═╡ 29bd8490-4e32-11eb-38b4-436ddc96af5a
@inline function get_heuristic(ᵐx, ᵐy, ᵐξₒ, boundary_poses) # ᵒξ
	# get map point wrt object frame, get x coord directly (equals projection)
end

# ╔═╡ a3f57948-4970-11eb-25f0-0d3d60aa888e
# return heuristic per object point
# return heuristic points for collection
# heuristic evaluation over map

# mutates free space elements (low val)
function evaluate_heuristic()
    # iterate over map and get heuristic values for all points (within low points)
end

# ╔═╡ 3723d85e-4e33-11eb-0ed4-1328588c58aa
# call upper function and plot

# ╔═╡ Cell order:
# ╟─3bda4224-4970-11eb-29ac-091d674c6763
# ╟─50f53aaa-49f5-11eb-0d5a-cb3e9c45649a
# ╠═701ff10c-4a1d-11eb-0b65-59b45daa23c2
# ╟─e6de0e36-496f-11eb-32e9-7f92c36296a3
# ╠═7307a8e6-4dde-11eb-26cb-6bdd3881b940
# ╠═754bbfc0-4ded-11eb-3719-d16818482c28
# ╠═51175c8e-4df1-11eb-057e-4741292fbe95
# ╠═98d7802a-4e65-11eb-2e34-abd3d757a2be
# ╠═aeadbe7e-4e35-11eb-1f95-67a0b327f61a
# ╠═aa19d822-4e65-11eb-0a43-05ff6cd19730
# ╠═db8bf968-4e2f-11eb-192b-8716d8109f4c
# ╠═29bd8490-4e32-11eb-38b4-436ddc96af5a
# ╠═a3f57948-4970-11eb-25f0-0d3d60aa888e
# ╠═3723d85e-4e33-11eb-0ed4-1328588c58aa
