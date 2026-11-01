#include "AP_QPOASES_Test.h"

#include <cmath>

namespace AP_QPOASES {

bool run_basic_test(BasicTestResult &result)
{
    constexpr qpOASES::int_t num_variables = 2;
    constexpr qpOASES::int_t num_constraints = 1;

    qpOASES::real_t hessian[num_variables * num_variables] = {
        1.0, 0.0,
        0.0, 1.0,
    };
    qpOASES::real_t gradient[num_variables] = {-2.0, -5.0};
    qpOASES::real_t constraint_matrix[num_constraints * num_variables] = {1.0, 1.0};
    qpOASES::real_t lower_bounds[num_variables] = {0.0, 0.0};
    qpOASES::real_t upper_bounds[num_variables] = {10.0, 10.0};
    qpOASES::real_t lower_constraint_bounds[num_constraints] = {-qpOASES::INFTY};
    qpOASES::real_t upper_constraint_bounds[num_constraints] = {3.0};

    qpOASES::QProblem problem(num_variables, num_constraints, qpOASES::HST_POSDEF);
    qpOASES::Options options;
    options.setToMPC();
    options.printLevel = qpOASES::PL_NONE;
    problem.setOptions(options);

    qpOASES::int_t iterations = 20;
    result.status = problem.init(
        hessian,
        gradient,
        constraint_matrix,
        lower_bounds,
        upper_bounds,
        lower_constraint_bounds,
        upper_constraint_bounds,
        iterations);
    result.iterations = iterations;

    if (result.status != qpOASES::SUCCESSFUL_RETURN) {
        return false;
    }

    result.status = problem.getPrimalSolution(result.solution);
    if (result.status != qpOASES::SUCCESSFUL_RETURN) {
        return false;
    }

    result.objective = problem.getObjVal();

    constexpr qpOASES::real_t tolerance = 1.0e-5;
    return std::fabs(result.solution[0]) <= tolerance &&
           std::fabs(result.solution[1] - 3.0) <= tolerance &&
           std::fabs(result.objective + 10.5) <= tolerance;
}

} // namespace AP_QPOASES
