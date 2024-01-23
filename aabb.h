#ifndef AABB_H
#define AABB_H
#include "interval.h"
#include "vec3.h"

class aabb {
  public:
    interval x, y, z;

    __device__ aabb() {} // The default AABB is empty, since intervals are empty by default.

    __device__ aabb(const interval& ix, const interval& iy, const interval& iz)
      : x(ix), y(iy), z(iz) { }

    __device__ aabb(const point3& a, const point3& b) {
        // Treat the two points a and b as extrema for the bounding box, so we don't require a
        // particular minimum/maximum coordinate order.
        x = interval(min(a[0],b[0]), max(a[0],b[0]));
        y = interval(min(a[1],b[1]), max(a[1],b[1]));
        z = interval(min(a[2],b[2]), max(a[2],b[2]));
    }

    __device__ aabb(const aabb& box0, const aabb& box1) {
        x = interval(box0.x, box1.x);
        y = interval(box0.y, box1.y);
        z = interval(box0.z, box1.z);
    }

    __device__ const interval& axis(int n) const {
        if (n == 1) return y;
        if (n == 2) return z;
        return x;
    }

    __device__ bool hit(const ray& r, interval ray_t) const {
        for (int a = 0; a < 3; a++) {
            auto invD = 1 / r.direction()[a];
            auto orig = r.origin()[a];

            auto t0 = (axis(a).min - orig) * invD;
            auto t1 = (axis(a).max - orig) * invD;

            if (invD < 0) {
                auto tmp = t1;
                t1 = t0;
                t0 = tmp;
            }
                
            if (t0 > ray_t.min) ray_t.min = t0;
            if (t1 < ray_t.max) ray_t.max = t1;

            if (ray_t.max <= ray_t.min)
                return false;
        }
        return true;
    }

    __device__ aabb pad() {
        // Return an AABB that has no side narrower than some delta, padding if necessary.
        double delta = 0.0001;
        interval new_x = (x.size() >= delta) ? x : x.expand(delta);
        interval new_y = (y.size() >= delta) ? y : y.expand(delta);
        interval new_z = (z.size() >= delta) ? z : z.expand(delta);

        return aabb(new_x, new_y, new_z);
    }
};

#endif