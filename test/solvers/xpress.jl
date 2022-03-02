using Xpress

XPRESS_OPTIMIZER = MOI.Bridges.full_bridge_optimizer(
    Xpress.Optimizer(), Float64
)
push!(optimizers_high_tol, XPRESS_OPTIMIZER)
push!(optimizers, XPRESS_OPTIMIZER)
