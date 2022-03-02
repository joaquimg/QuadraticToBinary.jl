using HiGHS

highs = MOI.instantiate(HiGHS.Optimizer, with_bridge_type = Float64)
MOI.set(highs, MOI.Silent(), true)
push!(optimizers, highs)

highs_high_tol = MOI.instantiate(HiGHS.Optimizer, with_bridge_type = Float64)
MOI.set(highs_high_tol, MOI.Silent(), true)
tol = 1e-9
# MOI.set(highs_high_tol, MOI.RawOptimizerAttribute("small_matrix_value"), 1e-12)
MOI.set(highs_high_tol, MOI.RawOptimizerAttribute("mip_feasibility_tolerance"), tol)
MOI.set(highs_high_tol, MOI.RawOptimizerAttribute("primal_feasibility_tolerance"), tol)
# MOI.set(highs_high_tol, MOI.RawOptimizerAttribute("presolve"), "off")
push!(optimizers_high_tol, highs_high_tol)