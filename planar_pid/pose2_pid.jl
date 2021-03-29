### A Pluto.jl notebook ###
# v0.12.21

using Markdown
using InteractiveUtils

# ╔═╡ 322e91e0-8ea2-11eb-30c5-23cad2905fe3
using Plots

# ╔═╡ d814107c-8ea1-11eb-2b0b-8f7489166764
begin
	ξ₀ = (0, 0)
	Δx = 1.0
	Δy = 1.0
	Δθ = π/4
	Δt = 1.0
	
	𝑣 = √(Δx^2 + Δy^2)/Δt
	θ̇ = Δθ/Δt
	ξs = []
end

# ╔═╡ 7ea2d5ea-8ea2-11eb-2cb6-01697203b8f1
0 + 0.1

# ╔═╡ 0bd2d148-8ea5-11eb-28f3-8b60f61a2804
md"
# Create a differential drive simulation!
"

# ╔═╡ 6b1bb20a-8ea5-11eb-35a5-67a53a05d7e0
R = 𝑣/θ̇

# ╔═╡ c9f18c6e-8ea5-11eb-020f-55f8dff5279e


# ╔═╡ Cell order:
# ╠═322e91e0-8ea2-11eb-30c5-23cad2905fe3
# ╠═d814107c-8ea1-11eb-2b0b-8f7489166764
# ╠═7ea2d5ea-8ea2-11eb-2cb6-01697203b8f1
# ╠═0bd2d148-8ea5-11eb-28f3-8b60f61a2804
# ╠═6b1bb20a-8ea5-11eb-35a5-67a53a05d7e0
# ╠═c9f18c6e-8ea5-11eb-020f-55f8dff5279e
