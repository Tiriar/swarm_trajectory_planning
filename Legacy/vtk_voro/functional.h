#ifndef _FUNCTIONAL_HPP_

#define _FUNCTIONAL_HPP_

#include <vector>
#include <string>

namespace tools {
  template <class Object> class Predicate {
    public:
      virtual bool apply(const Object& input) const = 0;
  };

  template <class From, class To> class Function {
    public:
      virtual To apply(const From& input) const = 0;
  };

  class Vectors {
    public:
      template <class Object> static std::vector<Object> filter(const std::vector<Object>& input, const Predicate<Object>& predicate);
      template <class From, class To> static std::vector<To> transform(const std::vector<From>& input, const Function<From, To>& function);
      template <class From, class To> static std::vector<std::vector<To> > transform(const std::vector<std::vector<From> >& input, const Function<From, To>& function);
      template <class Object> static std::vector<Object> fromArray(const Object* input, const unsigned int& size);
      template <class Object> static std::vector<std::vector<Object> > split(const std::vector<Object> &, const Predicate<Object>& predicate);
      template <class Object> static std::vector<Object> concat(const std::vector<std::vector<Object> > &);
  };

  class Predicates {
    public:
      class IS_EMPTY_STRING : public :: tools::Predicate<std::string> {
        public:
          bool apply(const std::string& input) const;
      };
      class IS_NOT_EMPTY_STRING : public :: tools::Predicate<std::string> {
        public:
          bool apply(const std::string& input) const;
      };
      template <class Object> class IS_NOT_EMPTY_VECTOR : public :: tools::Predicate<std::vector<Object> > {
        public:
	  bool apply(const std::vector<Object>& input) const;
      };
  };

  class Functions {
    public:
      template <class Object> class SIZE : public :: tools::Function<std::vector<Object>, unsigned int> {
        public:
          unsigned int apply(const std::vector<Object>& input) const;
      };
  };

}

#endif
