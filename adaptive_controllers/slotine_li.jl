### A Pluto.jl notebook ###
# v0.19.9

using Markdown
using InteractiveUtils

# ╔═╡ 0c41a6b8-eff2-11ec-083c-cd8a2b0bfd14
md"

# Slotine-Li Adaptive Controller

$$\mathbf{M}(\mathbf{q})\ddot{\mathbf{q}} + \mathbf{C}(\mathbf{q})\dot{\mathbf{q}} + \mathbf{B}(\mathbf{q})\dot{\mathbf{q}} + \nabla_{\mathbf{q}}P(\mathbf{q}) = Y(\mathbf{q}, \dot{\mathbf{q}}, \ddot{\mathbf{q}})\boldsymbol{\Theta}$$

"

# ╔═╡ c9435471-2965-49e2-8063-b6051e540895
# allow custom tracking signal (by drawing or functional input)
# solve multipart DE
# get the evolution of Θ
# get the evolution of tracking error

# ╔═╡ 00000000-0000-0000-0000-000000000001
PLUTO_PROJECT_TOML_CONTENTS = """
[deps]
"""

# ╔═╡ 00000000-0000-0000-0000-000000000002
PLUTO_MANIFEST_TOML_CONTENTS = """
# This file is machine-generated - editing it directly is not advised

julia_version = "1.7.2"
manifest_format = "2.0"

[deps]
"""

# ╔═╡ Cell order:
# ╟─0c41a6b8-eff2-11ec-083c-cd8a2b0bfd14
# ╠═c9435471-2965-49e2-8063-b6051e540895
# ╟─00000000-0000-0000-0000-000000000001
# ╟─00000000-0000-0000-0000-000000000002
