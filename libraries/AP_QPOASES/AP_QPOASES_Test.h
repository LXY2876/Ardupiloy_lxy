#pragma once

#include <qpOASES.hpp>

namespace AP_QPOASES {

struct BasicTestResult {
    qpOASES::returnValue status = qpOASES::RET_QP_NOT_SOLVED;
    qpOASES::real_t solution[2] = {};
    qpOASES::real_t objective = 0.0;
    qpOASES::int_t iterations = 0;
};

bool run_basic_test(BasicTestResult &result);

} // namespace AP_QPOASES
