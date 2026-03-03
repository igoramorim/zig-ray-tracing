const std = @import("std");

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Vec3 = vec3.Vec3;
const f3 = vec3.f3;
const Material = @import("material.zig").Material;
const Ray = @import("ray.zig").Ray;
const Interval = @import("interval.zig").Interval;

pub const HitRecord = struct {
    p: Point3 = Point3{ 0.0, 0.0, 0.0 },
    normal: Vec3 = Vec3{ 0.0, 0.0, 0.0 },
    mat: Material = undefined,
    t: f64 = undefined,
    front_face: bool = undefined,

    pub fn set_face_normal(self: *HitRecord, r: Ray, outward_normal: Vec3) void {
        // sets the hit record normal vector
        // NOTE: the parameter 'outward_normal' is assumed to have unit length

        self.front_face = vec3.dot(r.direction, outward_normal) < 0.0;
        if (self.front_face) {
            // ray is outside the geometry
            self.normal = outward_normal;
        } else {
            // ray is inside the geometry
            self.normal = -outward_normal;
        }
    }
};

pub const Hittable = struct {
    ptr: *const anyopaque,
    hit_fn: *const fn (ptr: *const anyopaque, r: Ray, rayT: Interval, rec: *HitRecord) bool,

    pub fn hit(self: Hittable, r: Ray, rayT: Interval, rec: *HitRecord) bool {
        return self.hit_fn(self.ptr, r, rayT, rec);
    }
};

pub const HittableList = struct {
    const Self = @This();

    objects: std.ArrayList(Hittable),

    pub fn init(allocator: std.mem.Allocator) HittableList {
        return HittableList{ .objects = std.ArrayList(Hittable).init(allocator) };
    }

    pub fn clear(self: *HittableList) void {
        self.objects.deinit();
    }

    pub fn add(self: *HittableList, obj: Hittable) !void {
        try self.objects.append(obj);
    }

    pub fn hittable(self: *const HittableList) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        var temp_rec = HitRecord{};
        var hit_anything = false;
        var closest_so_far = ray_t.max;

        for (self.objects.items) |obj| {
            if (obj.hit(r, Interval{ .min = ray_t.min, .max = closest_so_far }, &temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec.* = temp_rec;
            }
        }

        return hit_anything;
    }
};

pub const Sphere = struct {
    const Self = @This();

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

    pub fn hittable(self: *const Sphere) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

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
