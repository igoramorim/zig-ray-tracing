const std = @import("std");

const vec3 = @import("vec3.zig");
const f3 = vec3.f3;
const Vec3 = vec3.Vec3;
const Point3 = vec3.Point3;
const Material = @import("material.zig").Material;
const Ray = @import("ray.zig").Ray;
const Interval = @import("interval.zig").Interval;
const HitRecord = @import("hittable.zig").HitRecord;

pub const Sphere = struct {
    center: Ray,
    radius: f64,
    mat: Material = undefined,

    // stationary sphere
    pub fn init(center: Point3, radius: f64, mat: Material) Sphere {
        return Sphere{
            .center = Ray{ .origin = center, .direction = Vec3{ 0.0, 0.0, 0.0 } },
            .radius = @max(0.0, radius),
            .mat = mat,
        };
    }

    // moving sphere
    pub fn init_moving(center1: Point3, center2: Point3, radius: f64, mat: Material) Sphere {
        return Sphere{
            .center = Ray{ .origin = center1, .direction = center2 - center1 },
            .radius = @max(0.0, radius),
            .mat = mat,
        };
    }

    pub fn hit(self: Sphere, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const current_center = self.center.at(r.time);
        const oc: Vec3 = current_center - r.origin;
        const a: f64 = vec3.length_squared(r.direction);
        const h: f64 = vec3.dot(r.direction, oc);
        const c: f64 = vec3.length_squared(oc) - (self.radius * self.radius);

        const discriminant: f64 = (h * h) - (a * c);
        if (discriminant < 0.0) {
            return false;
        }

        const sqrtd: f64 = std.math.sqrt(discriminant);
        // find the nearest root that lies in the acceptable range
        var root: f64 = (h - sqrtd) / a;
        if (!ray_t.surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!ray_t.surrounds(root)) {
                return false;
            }
        }

        rec.t = root;
        rec.p = r.at(rec.t);
        const outward_normal: Vec3 = (rec.p - current_center) / f3(self.radius);
        rec.set_face_normal(r, outward_normal);
        rec.mat = self.mat;

        return true;
    }
};
