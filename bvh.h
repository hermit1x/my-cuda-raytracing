#ifndef BVHH
#define BVHH

#include "hittable.h"
#include "hittable_list.h"
#include "curand_kernel.h"
#include "thrust/sort.h"

class bvh_node : public hittable {
    public:
    __device__ bvh_node(size_t _start, size_t _end, bvh_node *f) {
        start = _start;
        end = _end;
        father = f;
    }

    __device__ bvh_node(const hittable_list& list, curandState *state) : bvh_node(list.list, 0, list.list_size, state) { }

    // 递归六层左右就会爆掉，因此改为循环模拟递归
    __device__ bvh_node(hittable **objects, size_t _start, size_t _end, curandState *state) {
        father = nullptr;
        start = _start; end = _end;
        bvh_node *x = this;
        int axis;
        while (true) {
            if (x == nullptr) break;

            axis = curand(state) % 3;
            auto comparator = (axis == 0) ? box_x_compare
                            : (axis == 1) ? box_y_compare
                                          : box_z_compare;
            
            size_t span = x->end - x->start;
            // printf("span %d\n", span);
            if (span == 1) {
                x->left = x->right = objects[x->start];
                x->vl = x->vr = true;
            } else if (span == 2) {
                if (comparator(objects[x->start], objects[x->start +1])) {
                    x->left = objects[x->start];
                    x->right = objects[x->start+1];
                }
                else {
                    x->left = objects[x->start+1];
                    x->right = objects[x->start];
                }
                x->vl = x->vr = true;
            } else {
                auto mid = x->start + span / 2;
                if (!x->vl && !x->vr) {
                    thrust::sort(objects + x->start, objects + x->end, comparator);
                    x->left = new bvh_node(x->start, mid, x);
                    x->vl = true;
                    x = (bvh_node *)x->left;
                    continue;
                }
                // 递归，改变x
                if (x->vl && !x->vr) {
                    x->right = new bvh_node(mid, x->end, x);
                    x->vr = true;
                    x = (bvh_node *)x->right;
                    continue;
                }
            }
            if (x->vl && x->vr) {
                x->bbox = aabb(x->left->bounding_box(), x->right->bounding_box());
                x = x->father;
            }
        }
    }
    
    // __device__ bvh_node(hittable **src_objects, size_t start, size_t end, curandState *state) {
        
    //     auto objects = src_objects; // Create a modifiable array of the source scene objects

    //     int axis = curand(state) % 3;
    //     auto comparator = (axis == 0) ? box_x_compare
    //                     : (axis == 1) ? box_y_compare
    //                                   : box_z_compare;

    //     size_t object_span = end - start;
    //     printf("bvh - dfs - len: %d\n", object_span);
    //     if (object_span == 1) {
    //         left = right = objects[start];
    //     } else if (object_span == 2) {
    //         if (comparator(objects[start], objects[start+1])) {
    //             left = objects[start];
    //             right = objects[start+1];
    //         } else {
    //             left = objects[start+1];
    //             right = objects[start];
    //         }
    //     } else {
    //         thrust::sort(objects + start, objects + end, comparator);

    //         auto mid = start + object_span/2;
    //         left = new bvh_node(objects, start, mid, state);
    //         right = new bvh_node(objects, mid, end, state);
    //     }

    //     bbox = aabb(left->bounding_box(), right->bounding_box());
    // }

    __device__ bool hit(const ray& r, interval ray_t, hit_record& rec) const override {
        if (!bbox.hit(r, ray_t))
            return false;

        bool hit_left = left->hit(r, ray_t, rec);
        bool hit_right = right->hit(r, interval(ray_t.min, hit_left ? rec.t : ray_t.max), rec);

        return hit_left || hit_right;
    }

    __device__ aabb bounding_box() const override { return bbox; }

  private:
    hittable *left;
    hittable *right;
    bvh_node *father;
    aabb bbox;
    size_t start, end;
    bool vl = false, vr = false;

    __device__ static bool box_compare(
        const hittable* a, const hittable* b, int axis_index
    ) {
        return a->bounding_box().axis(axis_index).min < b->bounding_box().axis(axis_index).min;
    }

    __device__ static bool box_x_compare (const hittable* a, const hittable* b) {
        return box_compare(a, b, 0);
    }

    __device__ static bool box_y_compare (const hittable* a, const hittable* b) {
        return box_compare(a, b, 1);
    }

    __device__ static bool box_z_compare (const hittable* a, const hittable* b) {
        return box_compare(a, b, 2);
    }
};

#endif