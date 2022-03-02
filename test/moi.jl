
function ncqcp1test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # Max 2x + y
    # s.t. x * y <= 4 (c)
    #      x, y >= 1

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    vc1 = MOI.add_constraint(model, x, MOI.GreaterThan(1.0))
    @test vc1.value == x.value
    vc2 = MOI.add_constraint(model, y, MOI.GreaterThan(1.0))
    @test vc2.value == y.value

    vc3 = MOI.add_constraint(model, x, MOI.LessThan(10.0))
    @test vc3.value == x.value
    vc4 = MOI.add_constraint(model, y, MOI.LessThan(10.0))
    @test vc4.value == y.value

    cf = 1.0 * x * y
    c = MOI.add_constraint(model, cf, MOI.LessThan(4.0))
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([2.0, 1.0], [x, y]), 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE

    @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 9.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), x) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.VariablePrimal(), y) ≈ 1.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 1.0 atol=atol rtol=rtol

end

function ncqcp1test_mod2(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # Max 2x + y
    # s.t. x * y <= 4 (c)
    #      x, y >= 1

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    # set upper fallback
    MOI.set(model, QuadraticToBinary.FallbackUpperBound(), 10.0)
    @test MOI.get(model, QuadraticToBinary.FallbackUpperBound()) == 10.0

    # set and unset lower fallback
    MOI.set(model, QuadraticToBinary.FallbackLowerBound(), 0.0)
    @test MOI.get(model, QuadraticToBinary.FallbackLowerBound()) == 0.0
    MOI.set(model, QuadraticToBinary.FallbackLowerBound(), nothing)
    @test MOI.get(model, QuadraticToBinary.FallbackLowerBound()) == -Inf

    vc1 = MOI.add_constraint(model, x, MOI.GreaterThan(1.0))
    @test vc1.value == x.value
    vc2 = MOI.add_constraint(model, y, MOI.GreaterThan(1.0))
    @test vc2.value == y.value

    cf = 1.0 * x * y
    c = MOI.add_constraint(model, cf, MOI.LessThan(4.0))
    @test MOI.get(model, MOI.NumberOfConstraints{
        MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([2.0, 1.0], [x, y]), 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE

    @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 9.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), x) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.VariablePrimal(), y) ≈ 1.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 1.0 atol=atol rtol=rtol

end

function ncqcp2test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # Find x, y
    # s.t. x * y == 4 (c)
    #      x * x == 4 (c2)
    #      x, y >= 0

    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.EqualTo{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    vc1 = MOI.add_constraint(model, x, MOI.GreaterThan(0.0))
    @test vc1.value == x.value
    vc2 = MOI.add_constraint(model, y, MOI.GreaterThan(0.0))
    @test vc2.value == y.value

    # begin new
    vc3 = MOI.add_constraint(model, x, MOI.LessThan(10.0))
    @test vc3.value == x.value
    vc4 = MOI.add_constraint(model, y, MOI.LessThan(10.0))
    @test vc4.value == y.value
    # end new

    cf = 1.0 * x * y
    c = MOI.add_constraint(model, cf, MOI.EqualTo(4.0))

    cf2 = 1.0 * x * x
    c2 = MOI.add_constraint(model, cf2, MOI.EqualTo(4.0))

    @test MOI.get(model, MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.EqualTo{Float64}}()) == 2

    @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c)
    @test cf2 ≈ MOI.get(model, MOI.ConstraintFunction(), c2)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.VariablePrimal(), x) ≈ 2.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.VariablePrimal(), y) ≈ 2.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), c2) ≈ 4.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 2.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 2.0 atol=atol rtol=rtol

end

function qp1test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # homogeneous quadratic objective
    # Min x^2 + xy + y^2 + yz + z^2
    # st  x + 2y + 3z >= 4 (c1)
    #     x +  y      >= 1 (c2)
    #     x, y, z \in R

    MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}())
    MOI.supports_constraint(model, MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    v = MOI.add_variables(model, 3)
    @test MOI.get(model, MOI.NumberOfVariables()) == 3
    x = v[1]
    y = v[2]
    z = v[3]

    # begin new
    for _v in v
        lb = MOI.add_constraint(model, _v, MOI.GreaterThan(-10.0))
        ub = MOI.add_constraint(model, _v, MOI.LessThan(10.0))
        @test lb.value == _v.value
        @test ub.value == _v.value
    end
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 3
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 3
    # end new

    cf1 = 1.0 * x + 2.0 * y + 3.0 * z
    c1 = MOI.add_constraint(model, cf1, MOI.GreaterThan(4.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}}()) == 1

    c2 = MOI.add_constraint(model, 1.0 * x + y, MOI.GreaterThan(1.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}}()) == 2

    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MIN_SENSE
    obj = 1.0 * x * x + 1.0 * x * y + 1.0 * y * y + 1.0 * y * z + 1.0 * z * z
    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}(), obj)

    @test obj ≈ MOI.get(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}())

    @test cf1 ≈ MOI.get(model, MOI.ConstraintFunction(), c1)

    @test MOI.GreaterThan(4.0) == MOI.get(model, MOI.ConstraintSet(), c1)


    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 13/7 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), v) ≈ [4/7, 3/7, 6/7] atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives
    # # [λ_c1 + λ_c2, 2λ_c1 + λ_c2, 3λ_c1] = [2 1 0; 1 2 1; 0 1 2] * x
    # #                                    = [11, 16, 15] / 7
    # # hence λ_c1 = 5/7 and λ_c2 = 6/7.
    # @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ 5/7 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), c2) ≈ 6/7 atol=atol rtol=rtol

end

function qp2test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # Same as `qp1` but with duplicate terms then change the objective and sense
    # simple quadratic objective
    # Min x^2 + xy + y^2 + yz + z^2
    # st  x + 2y + 3z >= 4 (c1)
    #     x +  y      >= 1 (c2)
    #     x, y, z \in R

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    v = MOI.add_variables(model, 3)
    @test MOI.get(model, MOI.NumberOfVariables()) == 3
    x = v[1]
    y = v[2]
    z = v[3]

    # begin new
    for _v in v
        lb = MOI.add_constraint(model, _v, MOI.GreaterThan(-10.0))
        ub = MOI.add_constraint(model, _v, MOI.LessThan(10.0))
        @test lb.value == _v.value
        @test ub.value == _v.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 3
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 3
    # end new

    c1f = 1.0 * x + 2.0 * y + 3.0 * z
    c1 = MOI.add_constraint(model, c1f, MOI.GreaterThan(4.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}}()) == 1

    c2f = 1.0 * x + y
    c2 = MOI.add_constraint(model, c2f, MOI.GreaterThan(1.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}}()) == 2

    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MIN_SENSE
    obj = MOI.ScalarQuadraticFunction(
        MOI.ScalarQuadraticTerm.([2.0, 0.5, 0.5, 2.0, 1.0, 1.0, 1.0], [v[1], v[1], v[1], v[2], v[2], v[3], v[3]], [v[1], v[2], v[2], v[2], v[3], v[3], v[3]]),
        MOI.ScalarAffineTerm.(0.0, v),
        0.0)
    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}(), obj)

    @test obj ≈ MOI.get(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}())

    @test c1f ≈ MOI.get(model, MOI.ConstraintFunction(), c1)

    @test MOI.GreaterThan(4.0) == MOI.get(model, MOI.ConstraintSet(), c1)


    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 13/7 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), v) ≈ [4/7,3/7,6/7] atol=atol rtol=rtol

    # if config.duals
    #     @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    #     @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ 5/7 atol=atol rtol=rtol
    #     @test MOI.get(model, MOI.ConstraintDual(), c2) ≈ 6/7 atol=atol rtol=rtol
    # end

    # change objective to Max -2(x^2 + xy + y^2 + yz + z^2)
    # First clear the objective, this is needed if a `Bridges.Objective.SlackBridge` is used.
    MOI.set(model, MOI.ObjectiveSense(), MOI.FEASIBILITY_SENSE)
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE
    obj2 = MOI.ScalarQuadraticFunction(
        MOI.ScalarQuadraticTerm.(
            [-4.0, -1.0, -1.0, -4.0, -2.0, -2.0, -2.0],
            [v[1], v[1], v[1], v[2], v[2], v[3], v[3]],
            [v[1], v[2], v[2], v[2], v[3], v[3], v[3]]),
        MOI.ScalarAffineTerm.(0.0, v),
        0.0)
    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}(), obj2)

    @test obj2 ≈ MOI.get(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}())

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ -2*13/7 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), v) ≈ [4/7, 3/7, 6/7] atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ 10/7 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), c2) ≈ 12/7 atol=atol rtol=rtol

end

function qp3test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # non-homogeneous quadratic objective
    #    minimize 2 x^2 + y^2 + xy + x + y + 1
    #       s.t.  x, y >= 0
    #             x + y = 1

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}())
    MOI.supports_constraint(model, MOI.VariableIndex, MOI.GreaterThan{Float64})
    MOI.supports_constraint(model, MOI.ScalarAffineFunction{Float64}, MOI.EqualTo{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)

    # begin new
    for w in [x, y]
        ub = MOI.add_constraint(model, w, MOI.LessThan(10.0))
        @test ub.value == w.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 2
    # end new

    c1 = MOI.add_constraint(model,
        1.0 * x + y,
        MOI.EqualTo(1.0)
    )

    vc1 = MOI.add_constraint(model, x, MOI.GreaterThan(0.0))
    # We test this after the creation of every `VariableIndex` constraint
    # to ensure a good coverage of corner cases.
    @test vc1.value == x.value
    vc2 = MOI.add_constraint(model, y, MOI.GreaterThan(0.0))
    @test vc2.value == y.value

    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    obj = MOI.ScalarQuadraticFunction(
        MOI.ScalarQuadraticTerm.([4.0, 2.0, 1.0], [x, y, x], [x, y, y]),
        MOI.ScalarAffineTerm.([1.0, 1.0], [x, y]),
        1.0
    )
    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarQuadraticFunction{Float64}}(), obj)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    h = MOI.get(model.optimizer, MOI.RawSolver())
    HiGHS.Highs_writeOptions(h, "options.txt")

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 2.875 atol=atol rtol=rtol
    @test MOI.get(model, MOI.VariablePrimal(), [x, y]) ≈ [0.25, 0.75] atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 0.25 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 0.75 atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives [λ_vc1 + λ_c1, λ_vc2 + λ_c1] = [4 1; 1 2] * x + [1, 1] = [11, 11] / 4
    # # since `vc1` and `vc2` are not active, `λ_vc1` and `λ_vc2` are
    # # zero so `λ_c1 = 11/4`.
    # @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ 11 / 4 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), vc1) ≈ 0 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), vc2) ≈ 0 atol=atol rtol=rtol

    #=
    # change back to linear
    #        max 2x + y + 1
    #       s.t.  x, y >= 0
    #             x + y = 1
    # (x,y) = (1,0), obj = 3
    # First clear the objective, this is needed if a `Bridges.Objective.SlackBridge` is used.
    MOI.set(model, MOI.ObjectiveSense(), MOI.FEASIBILITY_SENSE)
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    objf = MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([2.0, 1.0], [x, y]), 1.0)
    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), objf)

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 3.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.VariablePrimal(), [x,y]) ≈ [1.0, 0.0] atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 1.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 0.0 atol=atol rtol=rtol
=#
    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ -2 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), vc1) ≈ 0 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), vc2) ≈ 1 atol=atol rtol=rtol

end

function qcp1test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # quadratic constraint
    # Max x  + y
    # st -x  + y >= 0 (c1[1])
    #     x  + y >= 0 (c1[2])
    #     x² + y <= 2 (c2)
    # Optimal solution
    # x = 1/2, y = 7/4

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.VectorAffineFunction{Float64}, MOI.Nonnegatives)
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    # begin new
    for w in [x, y]
        lb = MOI.add_constraint(model, w, MOI.GreaterThan(-3.0))
        ub = MOI.add_constraint(model, w, MOI.LessThan(3.0))
        @test ub.value == w.value
        @test lb.value == w.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 2
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 2
    # end new

    c1 = MOI.add_constraint(model,
        MOI.VectorAffineFunction(
            MOI.VectorAffineTerm.(
                [1,1,2,2],
                MOI.ScalarAffineTerm.([-1.0,1.0,1.0,1.0], [x,y,x,y])),
                [0.0,0.0]),
        MOI.Nonnegatives(2))
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VectorAffineFunction{Float64}, MOI.Nonnegatives}()) == 1

    c2f = MOI.ScalarQuadraticFunction(
        [MOI.ScalarQuadraticTerm(2.0, x, x)],
        [MOI.ScalarAffineTerm(1.0, y)],
        0.0)
    c2 = MOI.add_constraint(model, c2f, MOI.LessThan(2.0))
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([1.0,1.0], [x,y]), 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE

    @test c2f ≈ MOI.get(model, MOI.ConstraintFunction(), c2) atol=atol rtol=rtol

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 2.25 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), [x, y]) ≈ [0.5, 1.75] atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c1) ≈ [5/4, 9/4] atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), c2) ≈ 2 atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives
    # # [-1, -1] = [-λ_c1[1] + λ_c1[2], λ_c1[1] + λ_c1[2]] + λ_c2 * ([2 0; 0 0] * x + [0, 1])
    # #          = [-λ_c1[1] + λ_c1[2] + λ_c2, λ_c1[1] + λ_c1[2] + λ_c2]
    # # By complementary slackness, we have `λ_c1 = [0, 0]` so
    # # `λ_c2 = -1`.
    # @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ [0, 0] atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), c2) ≈ -1 atol=atol rtol=rtol

    # try delete quadratic constraint and go back to linear

    # FIXME why is this commented ?
    # MOI.delete(model, c2)
    #
    # MOI.optimize!(model)
    #
    # @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL
    #
    # @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT
    #
    # @test MOI.get(model, MOI.ObjectiveValue()) ≈ 0.0 atol=atol rtol=rtol
end

function qcp2test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # Max x
    # s.t. x^2 <= 2 (c)

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64},MOI.LessThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 1

    # begin new
    for w in [x]
        lb = MOI.add_constraint(model, w, MOI.GreaterThan(-3.0))
        ub = MOI.add_constraint(model, w, MOI.LessThan(3.0))
        @test ub.value == w.value
        @test lb.value == w.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 1
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 1
    # end new

    cf = MOI.ScalarQuadraticFunction(
        [MOI.ScalarQuadraticTerm(2.0, x, x)],
        [MOI.ScalarAffineTerm(0.0, x)],
        0.0)
    c = MOI.add_constraint(model, cf, MOI.LessThan(2.0))
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, x)], 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE

    @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c) atol=atol rtol=rtol

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ √2 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), x) ≈ √2 atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 2 atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives
    # # -1 = λ * (2 * x) = λ * 2√2
    # # hence λ = -1/(2√2).
    # @test MOI.get(model, MOI.ConstraintDual(), c) ≈ -1 / (2 * √2) atol=atol rtol=rtol

end

function qcp3test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # Min -x
    # s.t. x^2 <= 2

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 1

    # begin new
    for w in [x]
        lb = MOI.add_constraint(model, w, MOI.GreaterThan(-3.0))
        ub = MOI.add_constraint(model, w, MOI.LessThan(3.0))
        @test ub.value == w.value
        @test lb.value == w.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 1
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 1
    # end new

    cf = MOI.ScalarQuadraticFunction(
        [MOI.ScalarQuadraticTerm(2.0, x, x)],
        MOI.ScalarAffineTerm{Float64}[],
        0.0)
    c = MOI.add_constraint(model, cf, MOI.LessThan(2.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(-1.0, x)], 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MIN_SENSE

    @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c) atol=atol rtol=rtol

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ -√2 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), x) ≈ √2 atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 2 atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives
    # # -1 = λ * (2 * x) = λ * 2√2
    # # hence λ = -1/(2√2).
    # @test MOI.get(model, MOI.ConstraintDual(), c) ≈ -1 / (2 * √2) atol=atol rtol=rtol

end

function _qcp4test_mod(model::MOI.ModelLike, config, less_than::Bool)
    atol = config.atol
    rtol = config.rtol
    # Max  x
    # s.t. x^2 + x * y + y^2 <= 3 (c)
    #      y == 1

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    quad_set = less_than ? MOI.LessThan(3.0) : MOI.GreaterThan(-3.0)
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, typeof(quad_set))

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    vc = MOI.add_constraint(model, y, MOI.EqualTo(1.0))
    @test vc.value == y.value

    # begin new
    for w in [x]
        lb = MOI.add_constraint(model, w, MOI.GreaterThan(-3.0))
        ub = MOI.add_constraint(model, w, MOI.LessThan(3.0))
        @test ub.value == w.value
        @test lb.value == w.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 1
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 1
    # end new

    cf = MOI.ScalarQuadraticFunction(
        MOI.ScalarQuadraticTerm.([2.0, 1.0, 2.0], [x, x, y], [x, y, y]),
        [MOI.ScalarAffineTerm(0.0, x)],
        0.0)
    if !less_than
        MOIU.operate!(-, Float64, cf)
    end
    c = MOI.add_constraint(model, cf, quad_set)
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, typeof(quad_set)}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
            MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, x)], 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE

    @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c) atol=atol rtol=rtol
    @test quad_set == MOI.get(model, MOI.ConstraintSet(), c)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 1.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), x) ≈ 1.0 atol=atol rtol=rtol
    @test MOI.get(model, MOI.VariablePrimal(), y) ≈ 1.0 atol=atol rtol=rtol

    @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ MOI.constant(quad_set) atol=atol rtol=rtol
    @test MOI.get(model, MOI.ConstraintPrimal(), vc) ≈ 1.0 atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives
    # # [-1, 0] = λ_c * [2 1; 1 2] * x + [0, λ_vc]
    # #         = [3λ_c, 3λ_c + λ_vc]
    # # hence `λ_c = -1/3` and `λ_vc = 1`.
    # λ_c = less_than ? -1/3 : 1/3
    # @test MOI.get(model, MOI.ConstraintDual(), c) ≈ λ_c atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), vc) ≈ 1 atol=atol rtol=rtol

end

qcp4test_mod(model::MOI.ModelLike, config) = _qcp4test_mod(model, config, true)
qcp5test_mod(model::MOI.ModelLike, config) = _qcp4test_mod(model, config, false)

function socp1test_mod(model::MOI.ModelLike, config)
    atol = config.atol
    rtol = config.rtol
    # min t
    # s.t. x + y >= 1 (c1)
    #      x^2 + y^2 <= t^2 (c2)
    #      t >= 0 (bound)

    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64})
    @test MOI.supports_constraint(model, MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64})
    @test MOI.supports_constraint(model, MOI.VariableIndex, MOI.GreaterThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    t = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 3

    # begin new
    for w in [x, y]
        lb = MOI.add_constraint(model, w, MOI.GreaterThan(-3.0))
        ub = MOI.add_constraint(model, w, MOI.LessThan(3.0))
        @test ub.value == w.value
        @test lb.value == w.value
    end
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 2
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.LessThan{Float64}}()) == 2
    MOI.add_constraint(model, t, MOI.LessThan(3.0))
    # end new

    c1f = MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([1.0, 1.0], [x,y]), 0.0)
    c1 = MOI.add_constraint(model, c1f, MOI.GreaterThan(1.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarAffineFunction{Float64}, MOI.GreaterThan{Float64}}()) == 1

    c2f = MOI.ScalarQuadraticFunction(
        MOI.ScalarQuadraticTerm.([2.0, 2.0, -2.0], [x, y, t], [x, y, t]),
        MOI.ScalarAffineTerm{Float64}[],
        0.0
    )
    c2 = MOI.add_constraint(model, c2f, MOI.LessThan(0.0))
    @test MOI.get(model,
        MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    bound = MOI.add_constraint(model, t, MOI.GreaterThan(0.0))
    @test bound.value == t.value
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.VariableIndex, MOI.GreaterThan{Float64}}()) == 3

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(),
        MOI.ScalarAffineFunction([MOI.ScalarAffineTerm(1.0, t)], 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MIN_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MIN_SENSE

    @test c1f ≈ MOI.get(model, MOI.ConstraintFunction(), c1)

    @test c2f ≈ MOI.get(model, MOI.ConstraintFunction(), c2)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

    MOI.optimize!(model)

    @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMAL

    @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

    @test MOI.get(model, MOI.ObjectiveValue()) ≈ 1/√2 atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), [x,y,t]) ≈ [0.5,0.5,1/√2] atol=atol rtol=rtol

    @test MOI.get(model, MOI.VariablePrimal(), [t,x,y,t]) ≈ [1/√2,0.5,0.5,1/√2] atol=atol rtol=rtol

    # @test MOI.get(model, MOI.DualStatus()) == MOI.FEASIBLE_POINT
    # # The dual constraint gives
    # # [1, 0, 0] = [λ_bound, λ_c1, λ_c1] + λ_c2 * [-2 0 0; 0 2 0; 0 0 2] * x
    # #           = [λ_bound - √2*λ_c2, λ_c1 + λ_c2, λ_c1 + λ_c2]
    # # and since `bound` is not active, `λ_bound = 0`.
    # # This gives `λ_c2 = -1/√2` and `λ_c1 = 1/√2`.
    # @test MOI.get(model, MOI.ConstraintDual(), c1) ≈ 1/√2 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), c2) ≈ -1/√2 atol=atol rtol=rtol
    # @test MOI.get(model, MOI.ConstraintDual(), bound) ≈ 0 atol=atol rtol=rtol

end