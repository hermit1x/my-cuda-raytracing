#ifndef SPHEREH
#define SPHEREH

#include "hittable.h"

class sphere: public hittable  {
    public:
        __device__ sphere() {}
        __device__ sphere(vec3 cen, float r, material *m) : center(cen), radius(r), mat(m)  
        {
            auto rvec = vec3(r, r, r);
            bbox = aabb(cen - rvec, cen + rvec);
        };
        __device__ virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const;
        __device__ aabb bounding_box() const override { return bbox; }
        vec3 center;
        float radius;
        material *mat;
        aabb bbox;
};

__device__ bool sphere::hit(const ray& r, interval ray_t, hit_record& rec) const {
    vec3 oc = r.origin() - center;
    float a = dot(r.direction(), r.direction());
    float b = dot(oc, r.direction());
    float c = dot(oc, oc) - radius*radius;
    float discriminant = b*b - a*c;
    if (discriminant > 0) {
        float temp = (-b - sqrt(discriminant))/a;
        if (temp < ray_t.max && temp > ray_t.min) {
            rec.t = temp;
            rec.p = r.point_at_parameter(rec.t);
            rec.normal = (rec.p - center) / radius;
            rec.mat = mat;
            return true;
        }
        temp = (-b + sqrt(discriminant)) / a;
        if (temp < ray_t.max && temp > ray_t.min) {
            rec.t = temp;
            rec.p = r.point_at_parameter(rec.t);
            rec.normal = (rec.p - center) / radius;
            rec.mat = mat;
            return true;
        }
    }
    return false;
}


#endif
