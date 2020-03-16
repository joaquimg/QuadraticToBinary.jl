using QuadraticToBinary, Test, MathOptInterface

const MOI = MathOptInterface
const MOIU = MOI.Utilities
const MOIT = MOI.Test
const MOIB = MOI.Bridges

optimizers = []
# include("solvers/gurobi.jl")
include("solvers/cbc.jl")

include("moi.jl")

const CONFIG_LOW_TOL = MOIT.TestConfig(atol = 1e-3, rtol = 1e-3)

@testset "From MOI ncqcp" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        ncqcp1test_mod(MODEL, CONFIG_LOW_TOL)
        ncqcp2test_mod(MODEL, CONFIG_LOW_TOL)
    end
end