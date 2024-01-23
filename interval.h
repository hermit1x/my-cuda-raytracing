#ifndef INTERVAL_H
#define INTERVAL_H

class interval {
  public:
    float min, max;

    __device__ interval() : min(__FLT_MAX__), max(__FLT_MIN__) {} // Default interval is empty

    __device__ interval(float _min, float _max) : min(_min), max(_max) {}

    __device__ interval(const interval& a, const interval& b)
      : min(fmin(a.min, b.min)), max(fmax(a.max, b.max)) {}

    __device__ bool contains(float x) const {
        return min <= x && x <= max;
    }

    __device__ bool surrounds(float x) const {
        return min < x && x < max;
    }

    __device__ float size() const {
        return max - min;
    }

    __device__ interval expand(double delta) const {
        auto padding = delta/2;
        return interval(min - padding, max + padding);
    }
    static const interval empty, universe;
};

const static interval empty   (__FLT_MAX__, __FLT_MIN__);
const static interval universe(__FLT_MIN__, __FLT_MAX__);

#endif