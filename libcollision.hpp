#ifndef LIBCOLLISION_HPP_
#define LIBCOLLISION_HPP_

#include <cmath>
#include <libvector.hpp>

namespace collision {

    template<typename T>
    struct rect2 {
        vector::vec2<T> pos;
        vector::vec2<T> size;
    };

    template<typename T>
    bool aabb_aabb(rect2<T> a, rect2<T> b) {
        return (a.pos.x < b.pos.x + b.size.x && 
                a.pos.x + a.size.x > b.pos.x &&
                a.pos.y < b.pos.y + b.size.y &&
                a.pos.y + a.size.y > b.pos.y);
    }

    template<typename T>
    bool point_aabb(vector::vec2<T> a, rect2<T> b) {
        return (a.x > b.x &&
                a.x < b.pos.x + b.size.x &&
                a.y > b.y &&
                a.y < b.pos.y + b.size.y);
    }

    template<typename T>
    bool ray_aabb(vector::vec2<T> r_pos, vector::vec2<T> r_dir, rect2<T> target, float &collision_time, vector::vec2<T> &normal_dir) {
        vector::vec2<T> entry_time = {
            target.pos.x - r_pos.x / r_dir.x,
            target.pos.y - r_pos.y / r_dir.y,
        };

        vector::vec2<T> exit_time = {
            target.pos.x + target.size.x - r_pos.x / r_dir.x,
            target.pos.y + target.size.y - r_pos.y / r_dir.y,
        };

        if(std::isnan(exit_time.y) || std::isnan(exit_time.y)) return false;
        if(std::isnan(entry_time.x) || std::isnan(entry_time.y)) return false; 

        if(entry_time.x > exit_time.x) std::swap(entry_time.x, exit_time.x);
        if(entry_time.y > exit_time.y) std::swap(entry_time.y, exit_time.y);

        if(entry_time.x > exit_time.x || entry_time.y > exit_time.y) return false;

        collision_time = std::max(entry_time.x, entry_time.y);
        float exit_hit_time = std::min(exit_time.x, exit_time.y);

        if(exit_hit_time < 0) return false;

        if(entry_time.x > entry_time.y) //latest entry is on x axis
            if(r_dir.x < 0.0f) //right side
                normal_dir = vector::vec2<T>{1.0f, 0.0f};
            else //left side
                normal_dir = vector::vec2<T>{-1.0f, 0.0f};
        else//latest entry is on y axis
            if(r_dir.y < 0.0f) //bottom side
                normal_dir = vector::vec2<T>{0.0f, 1.0f};
            else //top side
                normal_dir = vector::vec2<T>{0.0f, -1.0f};

        return true;
    }

    template<typename T>
    bool dynamic_aabb_aabb(rect2<T> a, vector::vec2<T> a_vel, rect2<T> b, float &collision_time, vector::vec2<T> &normal_dir) {
        if(a_vel.is_zero()) return false;

        rect2<T> b_expanded = {
            {b.pos.x - a.size.x, b.pos.y - a.size.y},
            {b.size.x + a.size.x * 2, b.size.y + a.size.y * 2}
        };

        if(ray_aabb(a.pos, a_vel, b_expanded, collision_time, normal_dir) && collision_time <= 1.0f) {
            return true;
        }

        return false;
    }
};

#endif
