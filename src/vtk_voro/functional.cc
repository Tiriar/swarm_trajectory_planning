#include "functional_def.h"

#include <string>

bool tools::Predicates::IS_EMPTY_STRING::apply(const std::string& input) const {
  return input.size() == 0;
}

bool tools::Predicates::IS_NOT_EMPTY_STRING::apply(const std::string& input) const {
  return input.size() != 0;
}

template class std::vector<int> tools::Vectors::filter(const std::vector<int> &, const tools::Predicate<int> &);
template class std::vector<std::string> tools::Vectors::filter(const std::vector<std::string> &, const tools::Predicate<std::string> &);
template class std::vector<std::vector<std::string> > tools::Vectors::filter(const std::vector<std::vector<std::string> > &, const tools::Predicate<std::vector<std::string> > &);

template class std::vector<double> tools::Vectors::transform(const std::vector<double> &, const tools::Function<double, double> &);
template class std::vector<std::vector<double> > tools::Vectors::transform(const std::vector<std::vector<double> > &, const tools::Function<double, double> &);

template class std::vector<std::string> tools::Vectors::fromArray(const std::string*, const unsigned int &);
template class std::vector<int> tools::Vectors::fromArray(const int*, const unsigned int &);
template class std::vector<double> tools::Vectors::fromArray(const double*, const unsigned int &);

template class std::vector<std::vector<std::string> > tools::Vectors::split(const std::vector<std::string> &, const tools::Predicate<std::string> &);

template class std::vector<std::string> tools::Vectors::concat(const std::vector<std::vector<std::string> > &);

template class tools::Functions::SIZE<std::string>;
template class tools::Predicates::IS_NOT_EMPTY_VECTOR<std::string>;
