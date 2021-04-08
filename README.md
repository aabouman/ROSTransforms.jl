# TF.jl

[![](https://img.shields.io/badge/docs-stable-blue.svg)](https://aabouman.github.io/TF.jl/)

This package was adapted from the work at [HiroIshida/RobotOS.jl](https://github.com/HiroIshida/RobotOS.jl/tree/add_tf), a fork of the [RobotOS.jl](https://github.com/jdlangs/RobotOS.jl) package.


## NOTE

Make sure you have built PyCall with the python distribution in which you
installed the tf libraries.

```julia
julia> using PyCall
julia> julia> ENV["PYTHON"]="/usr/bin/python2.7"
julia> using Pkg
julia> Pkg.build("PyCall")
```
