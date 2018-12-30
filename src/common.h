//
// Created by Roman Smirnov on 2018-12-30.
//

#ifndef EXTENDED_KALMAN_PROJECT_COMMON_H
#define EXTENDED_KALMAN_PROJECT_COMMON_H

//#include <vector>
#include <list>
#include <string>
#include <Eigen/Dense>

namespace kalman {

using std::list;

using Vector = Eigen::VectorXd;

using Matrix = Eigen::MatrixXd;

using Measurement = Eigen::VectorXd;

using State = Eigen::VectorXd;

}

#endif //EXTENDED_KALMAN_PROJECT_COMMON_H
