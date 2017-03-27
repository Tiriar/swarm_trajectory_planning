#include "doubles.h"

#include <cmath>
#include <limits>

double tools::Doubles::_precision = DOUBLES_PRECISION;

void tools::Doubles::setPrecision(const double& precision) {
  if(precision > 0) {
    _precision = precision;
  } else {
    setDefaultPrecision();
  }
}

bool tools::Doubles::equals(const double& value1, const double& value2) {
  if(value1 == std::numeric_limits<double>::infinity() && value2 == value1) return true;
  if(value1 == -std::numeric_limits<double>::infinity() && value2 == value1) return true;
  return std::abs(value1 - value2) <= precision();
}

bool tools::Doubles::greaterThan(const double& value1, const double& value2) {
  return value1 - value2 > precision();;
}
