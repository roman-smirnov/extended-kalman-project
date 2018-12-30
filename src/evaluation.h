//
// Created by Roman Smirnov on 2018-12-28.
//

#ifndef EXTENDED_KALMAN_PROJECT_EVALUATION_H
#define EXTENDED_KALMAN_PROJECT_EVALUATION_H

#include "common.h"

namespace kalman {


class Evaluation {

 public:
  Evaluation();
  virtual ~Evaluation();

  void AddGroundTrueMeasurement(Measurement measurement);

  void AddEstimatedMeasurement(Measurement measurement);

 private:
  list<Measurement> estimated_measurements;
  list<Measurement> true_measurements;

};

}

#endif //EXTENDED_KALMAN_PROJECT_EVALUATION_H
