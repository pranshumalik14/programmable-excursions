### A Pluto.jl notebook ###
# v0.14.3

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
	using TikzPictures;
	TikzPictures.standaloneWorkaround(true);
end

# ╔═╡ 4de32c03-45d8-4390-a977-b7661cd4453c
md"

# Running Instructions

Since this notebook makes use of Tikz diags, to run this notebook it is necessary to have the relevent tex packages installed. Therefore, please run the following commands to make sure you have all dependencies met:
```shell
sudo apt-get install texlive-latex-base texlive-binaries texlive-luatex texlive-latex-extra
```

"

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

# ╔═╡ 9ae5a438-9359-11eb-2f03-579a93583e6d
TikzPicture("
\\node (start) [startstop] {Start};
\\node (in1) [io, below of=start] {Input};
\\node (pro1) [process, below of=in1] {Process 1};
\\node (dec1) [decision, below of=pro1, yshift=-0.5cm] {Decision 1};
\\node (pro2a) [process, below of=dec1, yshift=-0.5cm] {Process 2a text text text text text text text text text text};
\\node (pro2b) [process, right of=dec1, xshift=2cm] {Process 2b};
\\node (out1) [io, below of=pro2a] {Output};
\\node (stop) [startstop, below of=out1] {Stop};

\\draw [arrow] (start) -- (in1);
\\draw [arrow] (in1) -- (pro1);
\\draw [arrow] (pro1) -- (dec1);
\\draw [arrow] (dec1) -- node[anchor=east] {yes} (pro2a);
\\draw [arrow] (dec1) -- node[anchor=south] {no} (pro2b);
\\draw [arrow] (pro2b) |- (pro1);
\\draw [arrow] (pro2a) -- (out1);
\\draw [arrow] (out1) -- (stop);
";
options="node distance=2cm", 
preamble="\\usepackage[utf8]{inputenc}
\\usetikzlibrary{shapes.geometric, arrows}
\\tikzstyle{startstop} = [rectangle, rounded corners, minimum width=3cm, minimum height=1cm,text centered, draw=black, fill=red!30]
\\tikzstyle{io} = [trapezium, trapezium left angle=70, trapezium right angle=110, minimum width=3cm, minimum height=1cm, text centered, draw=black, fill=blue!30]
\\tikzstyle{process} = [rectangle, minimum width=3cm, minimum height=1cm, text centered, text width=3cm, draw=black, fill=orange!30]
\\tikzstyle{decision} = [diamond, minimum width=3cm, minimum height=1cm, text centered, draw=black, fill=green!30]
\\tikzstyle{arrow} = [thick,->,>=stealth]")

# ╔═╡ de23b740-9164-11eb-1067-91f5035006ed
begin
	plot(; aspect_ratio=:equal);
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

# ╔═╡ 275ef943-ea9a-4809-a7a5-90178fa3d594
md"
Try doing the following below:

Size is $(@bind height Scrubbable(150:300; default=250))px by $(@bind width Scrubbable(600:680; default=680))px and every `100`px is $(@bind dist_per_100px Scrubbable(0.1:0.1:3; default=1)) meter(s).


$(@bind drawing HTML(\"\"\"
<div id=parent>
	<canvas id=canvas width=680px height=250px></canvas>
	<button id=clearButton>Clear</button>
	<button id=doneButton>Done</button>
</div>

<script>
	var canvasWidth = 680, canvasHeight = 250;
	const background = \"#f1f1f1\";
	
	const parentDiv = currentScript.previousElementSibling
	const c = parentDiv.querySelector(\"canvas\")
	const ctx = c.getContext(\"2d\");

	ctx.fillStyle = background;
	ctx.fillRect(0, 0, canvasWidth, canvasHeight);
	
	let drawing = false;
	parentDiv.value = [false, []];
	
	window.onmouseup = () => { drawing = false; };
	c.addEventListener('mousedown', () => drawing = true);
	c.addEventListener('mouseup', () => drawing = false);
	c.addEventListener('mousemove', (e) => {
		if(drawing) {
			if(parentDiv.value[1].length > 1) {
				const point = parentDiv.value[1][parentDiv.value[1].length - 1];
				ctx.moveTo(point[0][0], (canvasHeight - point[0][1]));
				ctx.lineTo(e.offsetX, e.offsetY);
				ctx.strokeStyle = 'black';
				ctx.lineWidth = 5;
				ctx.stroke();
			}

			parentDiv.value[0] = false;
			parentDiv.value[1].push([e.offsetX, (canvasHeight - e.offsetY)]);
			parentDiv.dispatchEvent(new CustomEvent(\"input\"));
		}
	});
	
	function clearCanvas(e) {
		ctx.beginPath();
		ctx.fillStyle = background;
		ctx.fillRect(0, 0, canvasWidth, canvasHeight);
		parentDiv.value = [false, []];
		parentDiv.dispatchEvent(new CustomEvent(\"input\"));
	}

	function readyOutput(e) {
		drawing = false;
		parentDiv.value[0] = true;
		parentDiv.dispatchEvent(new CustomEvent(\"input\"));
	}
	
	parentDiv.querySelector(\"#clearButton\").addEventListener('click', clearCanvas);
	parentDiv.querySelector(\"#doneButton\").addEventListener('click', readyOutput);
</script>
\"\"\"))
"

# ╔═╡ 405803a6-eadd-410c-9f2f-182cb85f63ca
begin
	if drawing[1] === true
		plot(; aspect_ratio=:equal, xlims=(0,680), ylims=(0,250));
		Vector{Point2}(Point2.(drawing[2])) |> plot_points
	end
end

# ╔═╡ 6788d28b-c550-4d45-8cc2-f45376b3d95f
# add tangent, cicle, and segment length code along with blocks for timestep and PID
# 	calculations.

# ╔═╡ 809bcc64-9974-4ef0-980e-e7255a430e5d
# to do: create a canvas element with reconfigurable size in PlutoUI style.
# Base.@kwdef struct Canvas
# 	height::Int64
# 	width::Int64
# 	px_to_dist::Real=1.0
# end

# ╔═╡ Cell order:
# ╟─4de32c03-45d8-4390-a977-b7661cd4453c
# ╟─871f906a-9158-11eb-3769-99159e457dd2
# ╟─9ae5a438-9359-11eb-2f03-579a93583e6d
# ╠═de23b740-9164-11eb-1067-91f5035006ed
# ╟─63c86eca-9176-11eb-1407-3bd19ccfcb7e
# ╟─275ef943-ea9a-4809-a7a5-90178fa3d594
# ╠═405803a6-eadd-410c-9f2f-182cb85f63ca
# ╠═6788d28b-c550-4d45-8cc2-f45376b3d95f
# ╟─809bcc64-9974-4ef0-980e-e7255a430e5d
# ╟─322e91e0-8ea2-11eb-30c5-23cad2905fe3
