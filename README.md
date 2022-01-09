# QuadraticToBinary.jl


| **Build Status** |
|:----------------:|
| [![Build Status][build-img]][build-url] [![Codecov branch][codecov-img]][codecov-url] |


[build-img]: https://github.com/joaquimg/QuadraticToBinary.jl/workflows/CI/badge.svg?branch=master
[build-url]: https://github.com/joaquimg/QuadraticToBinary.jl/actions?query=workflow%3ACI
[codecov-img]: http://codecov.io/github/joaquimg/QuadraticToBinary.jl/coverage.svg?branch=master
[codecov-url]: http://codecov.io/github/joaquimg/QuadraticToBinary.jl?branch=master


## Introduction

Non-convex quadratic programs are extremely hard to solve. This problem class can
be solved by Global Solvers such as [Couenne](https://projects.coin-or.org/Couenne).
Another possibility is to rely on binary expasion of products terms that appear in the
problem, in this case the problem is *approximated* and can be solved by off-the-shelf
MIP solvers such as [Cbc](https://github.com/JuliaOpt/Cbc.jl), [CPLEX](https://github.com/JuliaOpt/CPLEX.jl), [GLPK](https://github.com/JuliaOpt/GLPK.jl), [Gurobi](https://github.com/JuliaOpt/Gurobi.jl), [Xpress](https://github.com/JuliaOpt/Xpress.jl).

## Example

If one wants to solve the optimization problem with this package:

```julia
# Max 2x + y
# s.t. x * y <= 4 (c)
#      x, y >= 1
```

One should model as a quadratic program and simply wrap the solver with a
`QuadraticToBinary.Optimizer`, with one extra requirement: all variables appearing
in quadratic terms must be bounded above and below.

Therefore, the new model can be:


```julia
# Max 2x + y
# s.t. x * y <= 4 (c)
#      x, y >= 1
#      x, y <= 10
```

### JuMP with Cbc solver

```julia
using JuMP
using MathOptInterface
using QuadraticToBinary
using Cbc

const MOI = MathOptInterface
const MOIU = MOI.Utilities

const optimizer = MOI.Bridges.full_bridge_optimizer(
    MOIU.CachingOptimizer(
        MOIU.UniversalFallback(MOIU.Model{Float64}()),
        Cbc.Optimizer()), Float64)

model = Model(
    ()->QuadraticToBinary.Optimizer{Float64}(
        optimizer))

@variable(model, 1 <= x <= 10)
@variable(model, 1 <= y <= 10)

@constraint(model, c, x * y <= 4)

@objective(model, Max, 2x + y)

optimize!(model)

termination_status(model)

primal_status(model)

objective_value(model) # ≈ 9.0

@assert value(x) ≈ 4.0
@assert value(y) ≈ 1.0

@assert value(c) ≈ 4.0
```

### MathOptInterface with Cbc solver

```julia
using MathOptInterface
using QuadraticToBinary
const MOI = MathOptInterface
const MOIU = MOI.Utilities
using Cbc

const optimizer = MOI.Bridges.full_bridge_optimizer(
    MOIU.CachingOptimizer(
        MOIU.UniversalFallback(MOIU.Model{Float64}()),
        Cbc.Optimizer()), Float64)

model = QuadraticToBinary.Optimizer{Float64}(optimizer)

x = MOI.add_variable(model)
y = MOI.add_variable(model)

MOI.add_constraint(model, x, MOI.GreaterThan(1.0))
MOI.add_constraint(model, y, MOI.GreaterThan(1.0))

MOI.add_constraint(model, x, MOI.LessThan(10.0))
MOI.add_constraint(model, y, MOI.LessThan(10.0))

c = MOI.add_constraint(model, 1.0 * x * y, MOI.LessThan(4.0))

MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
    2.0 * x + y)
MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)

MOI.optimize!(model)

@assert MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

@assert MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

@assert MOI.get(model, MOI.ObjectiveValue()) ≈ 9.0

@assert MOI.get(model, MOI.VariablePrimal(), x) ≈ 4.0
@assert MOI.get(model, MOI.VariablePrimal(), y) ≈ 1.0

@assert MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 4.0
```

Note that duals are not available because the problem was approximated as a MIP.

## QuadraticToBinary.Optimizer Attributes

### Precision

It is possible to change the precision of the approximations to the number `val`,
for all variables:

```julia
MOI.set(model, QuadraticToBinary.GlobalVariablePrecision(), val)
```

Or, for each variable `vi`:

```julia
MOI.set(model, QuadraticToBinary.VariablePrecision(), vi, val)
```

The precision for each varible will be `val * (UB - LB)`. Where `UB` and `LB` are,
respectively, the upper and lower bound of the variable.

### Bounds

For the sake of simplicity, the following two attributes are made available:
`QuadraticToBinary.FallbackUpperBound` and `QuadraticToBinary.FallbackLowerBound`.
As usual, these can be get and set with the `MOI.get` and `MOI.set` methods.
These allow setting bounds used in variables that have no explicit upper bounds
and need to be expanded.


## Reference

For more details on the formulation applied here see this [paper](https://link.springer.com/article/10.1007/s10898-018-0728-9):

```
@article{andrade2019enhancing,
  title={Enhancing the normalized multiparametric disaggregation technique for mixed-integer quadratic programming},
  author={Andrade, Tiago and Oliveira, Fabricio and Hamacher, Silvio and Eberhard, Andrew},
  journal={Journal of Global Optimization},
  volume={73},
  number={4},
  pages={701--722},
  year={2019},
  publisher={Springer}
}
```

