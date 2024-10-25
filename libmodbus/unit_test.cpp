#include <iostream>
using namespace std;

std::pair<double, double> get_linear_velocities() {
        auto [rpmL, rpmR] = get_rpm();
        return {rpm_to_linear(rpmL), rpm_to_linear(-rpmR)};
    }
int main()
{
    
}