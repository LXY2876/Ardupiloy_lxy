#include "../AP_QPOASES_Test.h"

#include <cstdio>

int main()
{
    AP_QPOASES::BasicTestResult result;
    const bool passed = AP_QPOASES::run_basic_test(result);

    std::printf(
        "status=%d iterations=%d x=[%.9f, %.9f] objective=%.9f result=%s\n",
        static_cast<int>(result.status),
        static_cast<int>(result.iterations),
        static_cast<double>(result.solution[0]),
        static_cast<double>(result.solution[1]),
        static_cast<double>(result.objective),
        passed ? "PASS" : "FAIL");

    return passed ? 0 : 1;
}
