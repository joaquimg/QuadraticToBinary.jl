using QuadraticToBinary, Test, MathOptInterface

const MOI = MathOptInterface
const MOIU = MOI.Utilities
const MOIB = MOI.Bridges

optimizers = []
optimizers_high_tol = []
# include("solvers/gurobi.jl")
# include("solvers/xpress.jl")
# include("solvers/cbc.jl")
include("solvers/highs.jl")

@assert length(optimizers) > 0
@assert length(optimizers_high_tol) > 0

include("moi.jl")

const CONFIG_LOW_TOL = (atol = 1e-3, rtol = 1e-2)
const CONFIG_VERY_LOW_TOL = (atol = 5e-3, rtol = 5e-2)

function test_runtests(optimizer)
    model = QuadraticToBinary.Optimizer{Float64}(optimizer)
    MOI.set(model, MOI.Silent(), true)
    MOI.Test.runtests(
        model,
        MOI.Test.Config(
            atol = 1e-3, rtol = 1e-2, # LOW_TOL
            exclude = Any[
                MOI.ConstraintDual,
                MOI.DualObjectiveValue,
                MOI.ConstraintBasisStatus,
                MOI.VariableBasisStatus,
            ]
            ),
        exclude = String[
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
        ],
    )
    #=
        bounds required
    =#
    config = MOI.Test.Config(
        atol = 1e-3,
        rtol = 1e-2,
        exclude = Any[
            MOI.ConstraintDual,
            MOI.DualObjectiveValue
        ],
    )
    MOI.set(model, QuadraticToBinary.FallbackUpperBound(), +10.0)
    MOI.set(model, QuadraticToBinary.FallbackLowerBound(), -10.0)
    MOI.set(model, QuadraticToBinary.GlobalVariablePrecision(), 1e-5)
    MOI.set(model, MOI.Silent(), true)
    MOI.set(model, MOI.TimeLimitSec(), 20.0)
    MOI.Test.runtests(
        model,
        config,
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
            "test_quadratic_SecondOrderCone_basic",
            "test_quadratic_integration",
            # "test_quadratic_duplicate_terms",
        ]
    )
    config = MOI.Test.Config(
        atol = 5e-3,
        rtol = 5e-2,
        exclude = Any[
            MOI.ConstraintDual,
            MOI.DualObjectiveValue
        ],
    )
    MOI.Test.runtests(
        model,
        config,
        include = String[
            "test_quadratic_duplicate_terms",
        ]
    )
    return
end

@testset "MOI Unit" begin
    for opt in optimizers_high_tol
        @time test_runtests(opt)
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
    end
end

@testset "From MOI qp" begin
    for opt in optimizers_high_tol
        MODEL = QuadraticToBinary.Optimizer{Float64}(opt)
        MOI.set(MODEL, QuadraticToBinary.GlobalVariablePrecision(), 1e-6)
        qp3test_mod(MODEL, CONFIG_LOW_TOL)
    end
end
