using Gurobi

GUROBI_ENV = Gurobi.Env()
GUROBI_OPTIMIZER = MOI.Bridges.full_bridge_optimizer(
    Gurobi.Optimizer(GUROBI_ENV, OutputFlag=0), Float64
)
push!(optimizers, GUROBI_OPTIMIZER)
