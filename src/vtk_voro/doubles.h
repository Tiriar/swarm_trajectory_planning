#ifndef _DOUBLES_HPP_

#define _DOUBLES_HPP_

#ifndef DOUBLES_PRECISION
#define DOUBLES_PRECISION 1e-9
#endif

namespace tools {
  class Doubles {
    private:
      static double _precision;
    public:
      inline static double precision() {return _precision; }
      inline static void setDefaultPrecision() {_precision = DOUBLES_PRECISION; }
      static void setPrecision(const double& precision);
      static bool equals(const double& value1, const double& value2);
      static bool greaterThan(const double& value1, const double& value2);
  };
}

#endif
