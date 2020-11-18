### A Pluto.jl notebook ###
# v0.12.4

using Markdown
using InteractiveUtils

# ╔═╡ e1a82c1c-29d5-11eb-3413-69fff15bd618
md"

# Random Search Tree

An initial idea of finding a path from the start to goal configurations was to utilize random sampling instead of exhaustive search methods. This laid the foundation for Rapidly exploring Random Tree (RRT) that is now a popular method used in robotics.

Let's explore the idea.
"

# ╔═╡ b8b4c3e2-29da-11eb-3c9c-a9670cdc239f
md"

## Method


This should intutuively cover the space uniformly: uniform selection and expansion; what could go wrong?

"

# ╔═╡ c53e8a74-29da-11eb-3fc7-7bd3f914edce
md"

## Output

"

# ╔═╡ cf8fb27a-29da-11eb-0f31-4b2512289c7b
md"

## What's wrong

"

# ╔═╡ df11c170-29da-11eb-19b5-5328fd4399f8
md"

## Limiting probbility density

"

# ╔═╡ ea6aefce-29da-11eb-2d7f-f1f74887a519
md"

*source/inspriration:* [Rapidly exploring Random Topics by Steve M. LaValle](https://www.youtube.com/watch?v=OjNFjruZgaw&t=341s)

"

# ╔═╡ Cell order:
# ╠═e1a82c1c-29d5-11eb-3413-69fff15bd618
# ╠═b8b4c3e2-29da-11eb-3c9c-a9670cdc239f
# ╠═c53e8a74-29da-11eb-3fc7-7bd3f914edce
# ╠═cf8fb27a-29da-11eb-0f31-4b2512289c7b
# ╠═df11c170-29da-11eb-19b5-5328fd4399f8
# ╟─ea6aefce-29da-11eb-2d7f-f1f74887a519
