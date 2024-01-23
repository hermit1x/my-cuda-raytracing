#ifndef HITTABLELISTH
#define HITTABLELISTH

#include "hittable.h"
#include "aabb.h"

class hittable_list: public hittable  {
    public:
        __device__ hittable_list() {}
        __device__ hittable_list(hittable **l, int n) { 
            list = l; 
            list_size = n;
            for (int i = 0; i < n; ++i) bbox = aabb(bbox, l[i]->bounding_box()); 
        }
        __device__ virtual bool hit(const ray& r, interval ray_t, hit_record& rec) const;
        __device__ aabb bounding_box() const override { return bbox; }
        
        hittable **list;
        int list_size;
        aabb bbox;
};

__device__ bool hittable_list::hit(const ray& r, interval ray_t, hit_record& rec) const {
        hit_record temp_rec;
        bool hit_anything = false;
        float closest_so_far = ray_t.max;
        for (int i = 0; i < list_size; i++) {
            if (list[i]->hit(r, interval(ray_t.min, closest_so_far), temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec = temp_rec;
            }
        }
        return hit_anything;
}

#endif
