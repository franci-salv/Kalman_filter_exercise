#include <iostream>
#include <vector>
#include <cmath>

struct Step {
    double t;
    double meas;
};

int main() {
    std::vector<Step> data = {
        {0, 0.11}, {1, 1.02}, {2, 2.15}, {3, 2.95}, {4, 3.98},
        {5, 5.10}, {6, 6.01}, {7, 7.25}, {8, 7.95}, {9, 9.05},
        {10, 9.80}
    };

    // Expected correct behaviour (not a KF! just sanity check logic)
    for (auto &d : data) {
        double expected_vel = (data.back().meas - data.front().meas) /
                              (data.back().t - data.front().t); 

        if (std::abs(expected_vel - 0.98) > 0.2) {
            std::cout << "[WARNING] Your estimated velocity is off: " 
                      << expected_vel << "\n";
        }
    }

    std::cout << "If your filter converges to pos ~10 and vel ~1.0, it works.\n";
}
