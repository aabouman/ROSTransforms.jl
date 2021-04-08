using Literate, Documenter, EKF

# Literate.markdown("examples/SimpleExample.jl", "src/"; documenter=true)

Documenter.makedocs(
    sitename = "ROSTransforms.jl",
    source  = "src",
    build   = "build",
    clean   = true,
    doctest = true,
    # repo = "https://github.com/aabouman/ROSTransforms.jl",
    modules = [ROSTransforms],
    pages = ["Documentation" => "documentation.md"
             ]
)

deploydocs(;
    repo = "github.com/aabouman/ROSTransforms.jl",
    branch = "gh-pages",
    devurl = "docs",
)
