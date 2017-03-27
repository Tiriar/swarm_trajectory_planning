#ifndef _FUNCTIONALDEF_HPP_

#define _FUNCTIONALDEF_HPP_

#include "functional.h"

template <class Object> std::vector<Object> tools::Vectors::filter(const std::vector<Object>& input, const tools::Predicate<Object> &predicate) {
  std::vector<Object> filtered = std::vector<Object>();
  for(unsigned int i = 0; i < input.size(); i++) {
    if(predicate.apply(input[i])) {
      filtered.push_back(input[i]);
    }
  }
  return filtered;
}

template <class From, class To> std::vector<To> tools::Vectors::transform(const std::vector<From>& input, const tools::Function<From, To>& function) {
  std::vector<To> output = std::vector<To>();
  for(unsigned int i = 0; i < input.size(); i++) {
    output.push_back(function.apply(input[i]));
  }
  return output;
}

template <class From, class To> 
  std::vector<std::vector<To> > tools::Vectors::transform(const std::vector<std::vector<From> >& input, const tools::Function<From, To>& function) {
  std::vector<std::vector<To> > output = std::vector<std::vector<To> >();
  for(unsigned int i = 0; i < input.size(); i++) {
    output.push_back(tools::Vectors::transform(input[i], function));
  }
  return output;
}

template <class Object> std::vector<Object> tools::Vectors::fromArray(const Object* input, const unsigned int& size) {
  std::vector<Object> output = std::vector<Object>();
  for(unsigned int i = 0; i < size; i++) {
    output.push_back(input[i]);
  }
  return output;
}

template <class Object> std::vector<std::vector<Object> > tools::Vectors::split(const std::vector<Object>& input, const tools::Predicate<Object>& predicate) {
  std::vector<std::vector<Object> > output = std::vector<std::vector<Object> >();
  for(unsigned int i = 0; i < input.size(); i++) {
    std::vector<Object> subOutput = std::vector<Object>();
    while(i < input.size() && !predicate.apply(input[i])) {
      subOutput.push_back(input[i]);
      i++;
    }
    output.push_back(subOutput);
  }
  return output;
}

template <class Object> std::vector<Object> tools::Vectors::concat(const std::vector<std::vector<Object> >& input) {
  std::vector<Object> output = std::vector<Object>();
  for(unsigned int i = 0; i < input.size(); i++) {
    for(unsigned int j = 0; j < input[i].size(); j++) {
      output.push_back(input[i][j]);
    }
  }
  return output;
}

template <class Object> bool tools::Predicates::IS_NOT_EMPTY_VECTOR<Object>::apply(const std::vector<Object>& input) const {
  return input.size() != 0;
}

template <class Object> unsigned int tools::Functions::SIZE<Object>::apply(const std::vector<Object>& input) const {
  return input.size();
}

#endif
