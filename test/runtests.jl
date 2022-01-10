using QuadraticToBinary, Test, MathOptInterface

const MOI = MathOptInterface
const MOIU = MOI.Utilities
const MOIT = MOI.DeprecatedTest
const MOIB = MOI.Bridges

optimizers = []
# include("solvers/gurobi.jl")
include("solvers/cbc.jl")

include("moi.jl")

const CONFIG_LOW_TOL = MOIT.Config{Float64}(atol = 1e-3, rtol = 1e-2, duals = false, infeas_certificates = false)
const CONFIG_VERY_LOW_TOL = MOIT.Config{Float64}(atol = 5e-3, rtol = 5e-2, duals = false, infeas_certificates = false)
const CONFIG = MOIT.Config{Float64}(duals = false, infeas_certificates = false)

function test_runtests()
    optimizer = MOI.instantiate(Cbc.Optimizer, with_bridge_type = Float64)
    model = QuadraticToBinary.Optimizer{Float64}(optimizer)
    MOI.set(model, MOI.Silent(), true)
    MOI.Test.runtests(
        model,
        MOI.Test.Config(
            atol = 1e-3, rtol = 1e-2,
            exclude = Any[
                # MOI.DualStatus,
                # Cbc limitations
                MOI.ConstraintDual,
                MOI.DualObjectiveValue,
                MOI.ConstraintBasisStatus,
                MOI.VariableBasisStatus,
            ]
            ),
        exclude = String[
            # require bounds on variables:
            "test_quadratic_nonhomogeneous",
            "test_quadratic_nonconvex_constraint_integration",
            "test_quadratic_nonconvex_constraint_basic",
            "test_quadratic_integration",
            "test_quadratic_duplicate_terms",
            "test_quadratic_constraint_minimize",
            "test_quadratic_constraint_integration",
            "test_quadratic_constraint_basic",
            "test_quadratic_constraint_LessThan",
            "test_quadratic_constraint_GreaterThan",
            "test_quadratic_SecondOrderCone_basic",
            "test_objective_qp_ObjectiveFunction_zero_ofdiag",
            "test_objective_qp_ObjectiveFunction_edge_cases",
            "test_constraint_qcp_duplicate_off_diagonal",
            "test_constraint_qcp_duplicate_diagonal",
            # TODO: broken in Cbc:
            # https://github.com/jump-dev/Cbc.jl/blob/master/test/MOI_wrapper.jl
            # TODO(odow): bug in Cbc.jl
            "test_model_copy_to_UnsupportedAttribute",
            "test_model_ModelFilter_AbstractConstraintAttribute",
            # TODO(odow): bug in MOI
            "test_model_LowerBoundAlreadySet",
            "test_model_UpperBoundAlreadySet",
            # TODO(odow): upstream bug in Cbc
            "_Indicator_",
            "test_linear_SOS1_integration",
            "test_linear_SOS2_integration",
            "test_solve_SOS2_add_and_delete",
            # Can't prove infeasible.
            "test_conic_NormInfinityCone_INFEASIBLE",
            "test_conic_NormOneCone_INFEASIBLE",
            "test_solve_TerminationStatus_DUAL_INFEASIBLE",
        ],
    )
    #=
        Hard
    =#
    optimizer = MOI.instantiate(Cbc.Optimizer, with_bridge_type = Float64)
    model = QuadraticToBinary.Optimizer{Float64}(optimizer)
    MOI.set(model, QuadraticToBinary.FallbackUpperBound(), +10.0)
    MOI.set(model, QuadraticToBinary.FallbackLowerBound(), -10.0)
    MOI.set(model, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
    MOI.set(model, MOI.Silent(), true)
    MOI.set(model, MOI.TimeLimitSec(), 20.0)
    MOI.Test.runtests(
        model,
        MOI.Test.Config(
            atol = 5e-3,
            rtol = 5e-2,
            exclude = Any[
                MOI.ConstraintDual,
                MOI.DualObjectiveValue
            ],
        ),
        include = String[
            "test_quadratic_nonhomogeneous",
            "test_quadratic_nonconvex_constraint_integration",
            "test_quadratic_nonconvex_constraint_basic",
            "test_quadratic_constraint_minimize",
            "test_quadratic_constraint_integration",
            "test_quadratic_constraint_basic",
            "test_quadratic_constraint_LessThan",
            "test_quadratic_constraint_GreaterThan",
            "test_objective_qp_ObjectiveFunction_zero_ofdiag",
            "test_objective_qp_ObjectiveFunction_edge_cases",
            "test_constraint_qcp_duplicate_off_diagonal",
            "test_constraint_qcp_duplicate_diagonal",
        ]
    )
    #=
        more precise bounds
    =#
    MOI.set(model, QuadraticToBinary.FallbackUpperBound(), 1.0)
    MOI.set(model, QuadraticToBinary.FallbackLowerBound(), 0.0)
    MOI.set(model, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
    MOI.set(model, MOI.Silent(), false)
    MOI.set(model, MOI.TimeLimitSec(), 80.0)
    MOI.Test.runtests(
        model,
        MOI.Test.Config(
            atol = 5e-3,
            rtol = 5e-2,
            optimal_status = MOI.OPTIMAL,
            exclude = Any[
                MOI.ConstraintDual,
                MOI.DualObjectiveValue
            ],
        ),
        include = String[
            "test_quadratic_SecondOrderCone_basic",
            "test_quadratic_integration",
            "test_quadratic_duplicate_terms",
        ]
    )
    
    return
end
@testset "MOI Unit" begin
    @time test_runtests()
end

@testset "basic_constraint_tests" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOIT.basic_constraint_tests(MODEL, CONFIG)
    end
end

@testset "readme" begin
    for opt in optimizers

        MOI.empty!(opt)
        @test MOI.is_empty(opt)

        model = QuadraticToBinary.Optimizer{Float64}(opt)

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
    end
end

@testset "MOI Deprecated Unit" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOIT.unittest(MODEL, CONFIG, [
            "number_threads", # FIXME implement `MOI.NumberOfThreads`
            # INFEASIBLE_OR_UNBOUNDED instead of DUAL_INFEASIBLEÇ
            "solve_unbounded_model",
            # No quadratics (because of no bounds):
            "delete_soc_variables",
            "solve_qcp_edge_cases",
            "solve_qp_edge_cases",
            "solve_qp_zero_offdiag",
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
            MOIT.emptytest(MODEL)
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
            "int2",
        ])
    end
end

@testset "From MOI ncqcp" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        ncqcp1test_mod(MODEL, CONFIG_LOW_TOL)
        ncqcp1test_mod2(MODEL, CONFIG_LOW_TOL)
        ncqcp2test_mod(MODEL, CONFIG_LOW_TOL)
    end
end

@testset "From MOI qcp" begin
    for opt in optimizers
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOI.set(MODEL, QuadraticToBinary.GlobalVariablePrecision(), 1e-4)
        qcp1test_mod(MODEL, CONFIG_LOW_TOL)
        qcp2test_mod(MODEL, CONFIG_LOW_TOL)
        qcp3test_mod(MODEL, CONFIG_LOW_TOL)
        qcp4test_mod(MODEL, CONFIG_LOW_TOL)
        qcp5test_mod(MODEL, CONFIG_LOW_TOL)
        #
        MOI.set(MODEL, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
        socp1test_mod(MODEL, CONFIG_LOW_TOL)
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
