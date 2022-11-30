/*******************************************************************************
 * Copyright (c) 2022/7/11, Liao LunJia.
 * All rights reserved.
 *******************************************************************************/

#include "yqmrcf_common/filters/filters.h"

#include "yqmrcf_common/math_utilities.h"
namespace yqmrcf_filters {
/*---------------------------------RampFilter---------------------------------*/
template<typename T>
RampFilter<T>::RampFilter(T acc, T dt) {
  acc_ = acc;
  dt_ = dt;
  RampFilter::clear();
}

template<typename T>
void RampFilter<T>::input(T input_value) {
  last_value_ += minAbs(input_value - last_value_, acc_ * dt_);
}

template<typename T>
void RampFilter<T>::clear() {
  last_value_ = 0.;
}

template<typename T>
void RampFilter<T>::clear(T last_value) {
  last_value_ = last_value;
}

template<typename T>
void RampFilter<T>::setAcc(T acc) {
  acc_ = acc;
}

template<typename T>
T RampFilter<T>::output() {
  return last_value_;
}

template
class RampFilter<float>;
template
class RampFilter<double>;

/*---------------------------------LowPassFilter---------------------------------*/
template<typename T>
LowPassFilter<T>::LowPassFilter(T proportion) {
  proportion_ = proportion;
  LowPassFilter::clear();
}

template<typename T>
void LowPassFilter<T>::input(T input_value) {
  last_value_ = input_value * proportion_ + last_value_ * (1 - proportion_);
}

template<typename T>
void LowPassFilter<T>::clear() {
  last_value_ = 0.;
}

template<typename T>
void LowPassFilter<T>::clear(T last_value) {
  last_value_ = last_value;
}

template<typename T>
void LowPassFilter<T>::setProportion(T proportion) {
  proportion_ = proportion;
}

template<typename T>
T LowPassFilter<T>::output() {
  return last_value_;
}

template
class LowPassFilter<float>;
template
class LowPassFilter<double>;

}  // namespace yqmrcf_filters
