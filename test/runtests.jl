using QuadraticToBinary, Test, MathOptInterface

const MOI = MathOptInterface
const MOIU = MOI.Utilities
const MOIT = MOI.Test
const MOIB = MOI.Bridges

optimizers = []
# include("solvers/gurobi.jl")
include("solvers/cbc.jl")

include("moi.jl")

const CONFIG_LOW_TOL = MOIT.TestConfig(atol = 1e-3, rtol = 1e-2, duals = false, infeas_certificates = false)
const CONFIG_VERY_LOW_TOL = MOIT.TestConfig(atol = 5e-3, rtol = 5e-2, duals = false, infeas_certificates = false)
const CONFIG = MOIT.TestConfig(duals = false, infeas_certificates = false)

@testset "From MOI ncqcp" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        ncqcp1test_mod(MODEL, CONFIG_LOW_TOL)
        ncqcp2test_mod(MODEL, CONFIG_LOW_TOL)
    end
end

@testset "From MOI qp" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOI.set(MODEL, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
        qp1test_mod(MODEL, CONFIG_LOW_TOL)
        MOI.set(MODEL, QuadraticToBinary.GlobalVariablePrecision(), 1e-4)
        qp2test_mod(MODEL, CONFIG_VERY_LOW_TOL)
        qp3test_mod(MODEL, CONFIG_LOW_TOL)
    end
end

@testset "basic_constraint_tests" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOIT.basic_constraint_tests(MODEL, CONFIG)
    end
end

@testset "Unit" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOIT.unittest(MODEL, CONFIG, [
            "number_threads", # FIXME implement `MOI.NumberOfThreads`
            "solve_time", # FIXME implement `MOI.SolveTime`
            "solve_unbounded_model", # INFEASIBLE_OR_UNBOUNDED instead of DUAL_INFEASIBLE
            "delete_soc_variables", "solve_qcp_edge_cases", "solve_qp_edge_cases"  # No quadratics
        ])
    end
end

@testset "ModelLike" begin
    # @test MOI.get(CACHED, MOI.SolverName()) == "COIN Branch-and-Cut (Cbc)"
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        @testset "default_objective_test" begin
            MOIT.default_objective_test(MODEL)
        end
        @testset "default_status_test" begin
            MOIT.default_status_test(MODEL)
        end
        @testset "nametest" begin
            MOIT.nametest(MODEL)
        end
        @testset "validtest" begin
            MOIT.validtest(MODEL)
        end
        @testset "emptytest" begin
            # Requires VectorOfVariables
            # MOIT.emptytest(MODEL)
        end
        @testset "orderedindicestest" begin
            MOIT.orderedindicestest(MODEL)
        end
        @testset "copytest" begin
            # Requires VectorOfVariables
            # MOIT.copytest(MODEL, MOIU.CachingOptimizer(
            #     ModelForCachingOptimizer{Float64}(),
            #     Cbc.Optimizer()
            # ))
        end
    end
end

@testset "Continuous Linear" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOIT.contlineartest(MODEL, CONFIG)
    end
end

@testset "Integer Linear" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOIT.intlineartest(MODEL, CONFIG, [
            "indicator1", "indicator2", "indicator3", "indicator4",
            # TODO reenable when https://github.com/JuliaOpt/MathOptInterface.jl/issues/897 is resolved
            "semiconttest", "semiinttest",
        ])
    end
end
