#include "robotarm_kinematic.h"
#include <unistd.h>
#include <iostream>
#include <string>			//string lib
#include "Eigen/Eigen"
#include <cmath>

using namespace std;
using namespace Eigen;

void test(MatrixXd t)
{
    cout << t << endl;
}

int main(int argc, char *argv[])
{

    robotarm_kinematic rk(5);
    Isometry3d m = Isometry3d::Identity();

    double j_init[6] = {-0.0659, 1.54684, -1.58117, -0.034, -0.008, -0.02};
    m = rk.baseTend(j_init);
    cout << m.matrix() << endl;
    cout << m.rotation().eulerAngles(2,1,0) << endl;

    double j[6] = {0};
    rk.getQ(m, j_init, j);
    cout << j[0] << " " << j[1] << " " << j[2] << " " << j[3] << " " << j[4] << " " << j[5] << " " << endl;

    return 1;
}
