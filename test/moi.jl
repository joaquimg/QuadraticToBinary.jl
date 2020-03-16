function ncqcp2test_mod(model::MOI.ModelLike, config::MOIT.TestConfig)
    atol = config.atol
    rtol = config.rtol
    # Find x, y
    # s.t. x * y == 4 (c)
    #      x * x == 4 (c2)
    #      x, y >= 0

    # @test MOIU.supports_default_copy_to(model, #=copy_names=# false)
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.EqualTo{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    vc1 = MOI.add_constraint(model, MOI.SingleVariable(x), MOI.GreaterThan(0.0))
    @test vc1.value == x.value
    vc2 = MOI.add_constraint(model, MOI.SingleVariable(y), MOI.GreaterThan(0.0))
    @test vc2.value == y.value

    # begin new
    vc3 = MOI.add_constraint(model, MOI.SingleVariable(x), MOI.LessThan(10.0))
    @test vc3.value == x.value
    vc4 = MOI.add_constraint(model, MOI.SingleVariable(y), MOI.LessThan(10.0))
    @test vc4.value == y.value
    # end new

    cf = MOI.ScalarQuadraticFunction([MOI.ScalarAffineTerm(0.0, x)], [MOI.ScalarQuadraticTerm(1.0, x, y)], 0.0)
    c = MOI.add_constraint(model, cf, MOI.EqualTo(4.0))

    cf2 = MOI.ScalarQuadraticFunction([MOI.ScalarAffineTerm(0.0, x)], [MOI.ScalarQuadraticTerm(2.0, x, x)], 0.0)
    c2 = MOI.add_constraint(model, cf2, MOI.EqualTo(4.0))

    @test MOI.get(model, MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.EqualTo{Float64}}()) == 2

    if config.query
        @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c)
        @test cf2 ≈ MOI.get(model, MOI.ConstraintFunction(), c2)
    end

    if config.solve
        @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

        MOI.optimize!(model)

        @test MOI.get(model, MOI.TerminationStatus()) == config.optimal_status

        @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

        @test MOI.get(model, MOI.VariablePrimal(), x) ≈ 2.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.VariablePrimal(), y) ≈ 2.0 atol=atol rtol=rtol

        @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 4.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.ConstraintPrimal(), c2) ≈ 4.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 2.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 2.0 atol=atol rtol=rtol
    end
end

function ncqcp1test_mod(model::MOI.ModelLike, config::MOIT.TestConfig)
    atol = config.atol
    rtol = config.rtol
    # Max 2x + y
    # s.t. x * y <= 4 (c)
    #      x, y >= 1

    # @test MOIU.supports_default_copy_to(model, #=copy_names=# false)
    @test MOI.supports(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}())
    @test MOI.supports_constraint(model, MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64})

    MOI.empty!(model)
    @test MOI.is_empty(model)

    x = MOI.add_variable(model)
    y = MOI.add_variable(model)
    @test MOI.get(model, MOI.NumberOfVariables()) == 2

    vc1 = MOI.add_constraint(model, MOI.SingleVariable(x), MOI.GreaterThan(1.0))
    @test vc1.value == x.value
    vc2 = MOI.add_constraint(model, MOI.SingleVariable(y), MOI.GreaterThan(1.0))
    @test vc2.value == y.value

    vc3 = MOI.add_constraint(model, MOI.SingleVariable(x), MOI.LessThan(10.0))
    @test vc3.value == x.value
    vc4 = MOI.add_constraint(model, MOI.SingleVariable(y), MOI.LessThan(10.0))
    @test vc4.value == y.value

    cf = MOI.ScalarQuadraticFunction([MOI.ScalarAffineTerm(0.0, x)], [MOI.ScalarQuadraticTerm(1.0, x, y)], 0.0)
    c = MOI.add_constraint(model, cf, MOI.LessThan(4.0))
    @test MOI.get(model, MOI.NumberOfConstraints{MOI.ScalarQuadraticFunction{Float64}, MOI.LessThan{Float64}}()) == 1

    MOI.set(model, MOI.ObjectiveFunction{MOI.ScalarAffineFunction{Float64}}(), MOI.ScalarAffineFunction(MOI.ScalarAffineTerm.([2.0, 1.0], [x, y]), 0.0))
    MOI.set(model, MOI.ObjectiveSense(), MOI.MAX_SENSE)
    @test MOI.get(model, MOI.ObjectiveSense()) == MOI.MAX_SENSE

    if config.query
        @test cf ≈ MOI.get(model, MOI.ConstraintFunction(), c)
    end

    if config.solve
        @test MOI.get(model, MOI.TerminationStatus()) == MOI.OPTIMIZE_NOT_CALLED

        MOI.optimize!(model)

        @test MOI.get(model, MOI.TerminationStatus()) == config.optimal_status

        @test MOI.get(model, MOI.PrimalStatus()) == MOI.FEASIBLE_POINT

        @test MOI.get(model, MOI.ObjectiveValue()) ≈ 9.0 atol=atol rtol=rtol

        @test MOI.get(model, MOI.VariablePrimal(), x) ≈ 4.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.VariablePrimal(), y) ≈ 1.0 atol=atol rtol=rtol

        @test MOI.get(model, MOI.ConstraintPrimal(), c) ≈ 4.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.ConstraintPrimal(), vc1) ≈ 4.0 atol=atol rtol=rtol
        @test MOI.get(model, MOI.ConstraintPrimal(), vc2) ≈ 1.0 atol=atol rtol=rtol
    end
end