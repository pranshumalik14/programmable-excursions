### A Pluto.jl notebook ###
# v0.12.21

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

# ╔═╡ 322e91e0-8ea2-11eb-30c5-23cad2905fe3
begin
	include("../object_scanning/misc_utils.jl");
	using PlutoUI;
	using Plots;
	using Statistics;
	using LinearAlgebra;
	using Images;
end

# ╔═╡ 871f906a-9158-11eb-3769-99159e457dd2
md"

# Discrete-time PID Controller for `Pose2`

- What is the path we are getting? (time-stamped poses... get velocity from that):
  + an image of a path: SVG diagram of $\mathbf{\xi}(t)$
- What we send as control, what happens, path following right now; Root cause in prev design: not depending on the 𝑣 and θ̇ and rather calculated on the fly: jerky behaviour, more error, and weird loose control loops (no new command generated or something)...


A `Twist2` control signal, $\mathbf{u}$, comprising of 2 control actions -- linear and angular velocities -- is sent to the motor driver to move the robot accordingly.

$\mathbf{u} = \left[\mathbf{v} \quad \mathbf{\omega}\right]^{\intercal}$

A control action, $\mathbf{c}$, at timestep $n$, is governed by a recursive relation

> $\mathbf{c}_n = \mathbf{c}_{n-1} + \underbrace{k_p\Delta{\mathbf{c}_n} + k_d\frac{\Delta{\mathbf{c}_n}}{\Delta{t_n}} + k_i\left(\Delta{\mathbf{c}_n}\Delta{t_n} + \mathbf{I}_{n-1}\right)}_{= \Delta{\mathbf{c}}(n)}$ 
> $\implies \mathbf{c}_n = \mathbf{c}_{n-1} + \Delta{\mathbf{c}}(n)$

where,
- past integrated error, $\mathbf{I}_{n-1} = 2$
- intial control action, $\mathbf{c}_0 = 0$
- current difference in linear velocity, $\Delta{\mathbf{v}_n} := \mathbf{v}^{\text{ref}}_n - \mathbf{v}_{n-1}$
- current difference in angular velocity, $\Delta{\mathbf{\omega}_n} := \mathbf{\omega}^{\text{proj}}_n - \mathbf{\omega}_{n-1}$
- how about $\Delta{\mathbf{c}_i} := \mathbf{c}^{\text{proj}}_i - \mathbf{c}^{\text{ref}}_i$
- previous differnces from timestep i in speed are simply the differences between 
- time difference, $\Delta{t} := 2$
- Ki, kp, kd are the pid gains respectively

SVG diagram explaining things and the algorithm: $\mathbf{\omega}^{\text{proj}}$ etc.

Therefore, at each timestep, the following steps happen in order:
- Get current $^{\text{map}}{\mathbf{\xi}}_{\text{base-link}}$ from localization
- Update integrator
- Get new control action, $u$
- Send control

Note, $\theta$ is projected as we can be at any pose away from the reference pose and the calulcated pose. We need to drive towards reference at each time point as so by keeping speed as a pivot (const), we reculate theta dot.

"

# ╔═╡ de23b740-9164-11eb-1067-91f5035006ed
begin
	plot(; aspect_ratio=:equal)
	plot_poses([Pose2(3,2.2, 5*π/3), Pose2(3,2.2, π/2), 𝑍()]; color="green", α=0.1)
	plot_pose(Pose2(1,2, π))
	plot_pose(Pose2(1,2, π/2))
	plot_pose(Pose2(1,2, 4π/3))
end

# ╔═╡ 63c86eca-9176-11eb-1407-3bd19ccfcb7e
md"

- Addition of noise for all of above. make variable intensity + spread.
- Addition of measurements.jl to get the error plots of system and convergence criteria over single timesteps.
- Graphs of convergence of position and orientation over time while trying to follow path (just plot error over time).
- Animation
- A static plot of the entire path with varying alpha to show time.

"

# ╔═╡ 10181cbc-928c-11eb-0d11-fbab42ffb2c0
@bind drawing HTML("""
<div id=parent>
	<canvas id=canvas width=680px height=200px></canvas>
	<button id=clearButton>clear</button>
</div>
	
<script>
	const canvasWidth = 680, canvasHeight = 200, background = "#f1f1f1";
	
	const parentDiv = currentScript.closest('pluto-output').querySelector("div#parent")

	
	const c = document.getElementById("canvas");
	const ctx = c.getContext("2d");
	ctx.fillStyle = background;
	ctx.fillRect(0, 0, canvasWidth, canvasHeight);
	
	let drawing = false;
	
	c.addEventListener('mousedown', () => drawing = true);
	c.addEventListener('mouseup', () => drawing = false);
	c.addEventListener('mousemove', (e) => {
		if(drawing) {
			ctx.beginPath();
			ctx.arc(e.offsetX, e.offsetY, 4, 0, 2 * Math.PI);
			ctx.fillStyle = "#010101";
			ctx.fill();
				
			parentDiv.value = ctx.getImageData(0, 0, canvasWidth, canvasHeight).data;
			parentDiv.dispatchEvent(new CustomEvent("input"));
		}
	});
	
	function clearCanvas(e) {
		ctx.fillStyle = background;
		ctx.fillRect(0, 0, canvasWidth, canvasHeight);
		parentDiv.value = ctx.getImageData(0, 0, canvasWidth, canvasHeight).data;
		parentDiv.dispatchEvent(new CustomEvent("input"));
	}
	
	document.getElementById("clearButton").addEventListener('click', clearCanvas);
</script>
""")

# ╔═╡ 1fd8e246-928c-11eb-2961-f19871ec7c23
begin
	img = Vector{RGBA{N0f8}}()
	for i ∈ 1:4:length(drawing)
		push!(img, RGBA(reinterpret(N0f8, drawing[i:i+3])...))
	end
	img = reshape(img, 680, 200)
	img = imrotate(img, π/2)
	reverse(img; dims=2)
end

# ╔═╡ Cell order:
# ╟─871f906a-9158-11eb-3769-99159e457dd2
# ╠═de23b740-9164-11eb-1067-91f5035006ed
# ╟─63c86eca-9176-11eb-1407-3bd19ccfcb7e
# ╟─10181cbc-928c-11eb-0d11-fbab42ffb2c0
# ╠═1fd8e246-928c-11eb-2961-f19871ec7c23
# ╟─322e91e0-8ea2-11eb-30c5-23cad2905fe3
