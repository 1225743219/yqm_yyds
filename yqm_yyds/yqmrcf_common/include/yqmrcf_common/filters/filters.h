/*******************************************************************************
 * Copyright (c) 2022/7/11, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/
#pragma once

namespace yqmrcf_filters {
template<typename T>
class Filter {
 public:
  Filter() = default;
  virtual ~Filter() = default;
  virtual void input(T input_value) = 0;
  virtual T output() = 0;
  virtual void clear() = 0;
};

/*---------------------RampFilter---------------------------*/
template<typename T>
class RampFilter : public Filter<T> {
 public:
  RampFilter(T acc, T dt);
  ~RampFilter() = default;
  void input(T input_value);
  void clear();
  void clear(T last_value);
  void setAcc(T acc);  // without clear.
  T output();

 private:
  T last_value_;
  T acc_;
  T dt_;
};

/*---------------------LowPassFilter---------------------------*/
template<typename T>
class LowPassFilter : public Filter<T> {
 public:
  explicit LowPassFilter(T proportion);
  ~LowPassFilter() = default;
  void input(T input_value);
  void clear();
  void clear(T last_value);
  void setProportion(T proportion);  // without clear.
  T output();

 private:
  T last_value_;
  T proportion_;
  T dt_;
};

}  // namespace yqmrcf_filters
