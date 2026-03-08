const std = @import("std");

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Vec3 = vec3.Vec3;
const f3 = vec3.f3;
const Material = @import("material.zig").Material;
const Ray = @import("ray.zig").Ray;
const Interval = @import("interval.zig").Interval;
const common = @import("common.zig");

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
    bounding_box_fn: *const fn (ptr: *const anyopaque) AABB,

    pub fn hit(self: Hittable, r: Ray, rayT: Interval, rec: *HitRecord) bool {
        return self.hit_fn(self.ptr, r, rayT, rec);
    }

    pub fn bounding_box(self: Hittable) AABB {
        return self.bounding_box_fn(self.ptr);
    }
};

pub const HittableList = struct {
    const Self = @This();

    objects: std.ArrayList(Hittable),
    bbox: AABB = undefined,

    pub fn init(allocator: std.mem.Allocator) HittableList {
        return HittableList{ .objects = std.ArrayList(Hittable).init(allocator) };
    }

    pub fn clear(self: *HittableList) void {
        self.objects.deinit();
    }

    pub fn add(self: *HittableList, obj: Hittable) !void {
        try self.objects.append(obj);
        self.bbox = AABB.init_from_boxes(self.bbox, obj.bounding_box());
    }

    pub fn hittable(self: *const HittableList) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
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

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};

pub const Sphere = struct {
    const Self = @This();

    center: Ray,
    radius: f64,
    mat: Material,
    bbox: AABB,

    // stationary sphere
    pub fn init(center: Point3, radius: f64, mat: Material) Sphere {
        const r = @max(0.0, radius);
        const rvec = f3(r);

        return Sphere{
            .center = Ray{ .origin = center, .direction = Vec3{ 0.0, 0.0, 0.0 } },
            .radius = r,
            .mat = mat,
            .bbox = AABB.init_from_points(center - rvec, center + rvec),
        };
    }

    // moving sphere
    pub fn init_moving(center1: Point3, center2: Point3, radius: f64, mat: Material) Sphere {
        const r = @max(0.0, radius);
        const rvec = f3(r);
        const center = Ray{ .origin = center1, .direction = center2 - center1 };
        const box1 = AABB.init_from_points(center.at(0) - rvec, center.at(0) + rvec);
        const box2 = AABB.init_from_points(center.at(1) - rvec, center.at(1) + rvec);

        return Sphere{
            .center = center,
            .radius = r,
            .mat = mat,
            .bbox = AABB.init_from_boxes(box1, box2),
        };
    }

    pub fn hittable(self: *const Sphere) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
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

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};

pub const AABB = struct {
    x: Interval = Interval{},
    y: Interval = Interval{},
    z: Interval = Interval{},

    pub fn init_from_points(a: Point3, b: Point3) AABB {
        const x = if (a[0] <= b[0]) Interval{ .min = a[0], .max = b[0] } else Interval{ .min = b[0], .max = a[0] };
        const y = if (a[1] <= b[1]) Interval{ .min = a[1], .max = b[1] } else Interval{ .min = b[1], .max = a[1] };
        const z = if (a[2] <= b[2]) Interval{ .min = a[2], .max = b[2] } else Interval{ .min = b[2], .max = a[2] };

        return AABB{ .x = x, .y = y, .z = z };
    }

    pub fn init_from_boxes(box0: AABB, box1: AABB) AABB {
        return AABB{
            .x = Interval.init_from_intervals(box0.x, box1.x),
            .y = Interval.init_from_intervals(box0.y, box1.y),
            .z = Interval.init_from_intervals(box0.z, box1.z),
        };
    }

    pub fn empty() AABB {
        return AABB{ .x = Interval.empty(), .y = Interval.empty(), .z = Interval.empty() };
    }

    pub fn universe() AABB {
        return AABB{ .x = Interval.universe(), .y = Interval.universe(), .z = Interval.universe() };
    }

    pub fn hit(self: AABB, r: Ray, ray_t: Interval, _: *HitRecord) bool {
        const ray_orig = r.origin;
        const ray_dir = r.direction;
        var local_ray_t = ray_t;

        var axis: u8 = 0;
        while (axis < 3) : (axis = axis + 1) {
            const ax = self.axis_interval(axis);
            const adinv: f64 = 1.0 / ray_dir[axis];

            const t0 = (ax.min - ray_orig[axis]) * adinv;
            const t1 = (ax.max - ray_orig[axis]) * adinv;

            if (t0 < t1) {
                if (t0 > local_ray_t.min) local_ray_t.min = t0;
                if (t1 < local_ray_t.max) local_ray_t.max = t1;
            } else {
                if (t1 > local_ray_t.min) local_ray_t.min = t1;
                if (t0 < local_ray_t.max) local_ray_t.max = t0;
            }

            if (local_ray_t.max <= local_ray_t.min) return false;
        }

        return true;
    }

    pub fn axis_interval(self: AABB, axis: u8) Interval {
        if (axis == 0) return self.x;
        if (axis == 1) return self.y;
        return self.z;
    }

    pub fn longest_axis(self: AABB) u8 {
        if (self.x.size() > self.y.size()) {
            return if (self.x.size() > self.z.size()) 0 else 2;
        }
        return if (self.y.size() > self.z.size()) 1 else 2;
    }
};

pub const BVHNode = struct {
    const Self = @This();

    left: Hittable,
    right: Hittable,
    bbox: AABB,

    pub fn init(allocator: std.mem.Allocator, list: HittableList) !*BVHNode {
        return build(allocator, list.objects.items);
    }

    fn build(allocator: std.mem.Allocator, objects: []Hittable) !*BVHNode {
        const span = objects.len;
        var bbox = AABB.empty();

        for (0..span) |i| {
            bbox = AABB.init_from_boxes(bbox, objects[i].bounding_box());
        }

        const axis: u8 = bbox.longest_axis();
        const comparator = struct {
            var current_axis: u8 = 0;
            fn func(_: void, a: Hittable, b: Hittable) bool {
                const axis_interval_a = a.bounding_box().axis_interval(current_axis);
                const axis_interval_b = b.bounding_box().axis_interval(current_axis);
                return axis_interval_a.min < axis_interval_b.min;
            }
        };
        comparator.current_axis = axis;

        var left: Hittable = undefined;
        var right: Hittable = undefined;

        if (span == 1) {
            left = objects[0];
            right = objects[0];
        } else if (span == 2) {
            left = objects[0];
            right = objects[1];
        } else {
            std.mem.sort(Hittable, objects, {}, comparator.func);
            const mid = span / 2;
            const left_node = try build(allocator, objects[0..mid]);
            const right_node = try build(allocator, objects[mid..]);
            left = left_node.hittable();
            right = right_node.hittable();
        }

        const node = try allocator.create(BVHNode);
        node.* = BVHNode{
            .left = left,
            .right = right,
            .bbox = bbox,
        };

        return node;
    }

    pub fn hittable(self: *const BVHNode) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        if (!self.bbox.hit(r, ray_t, rec)) return false;

        const hit_left = self.left.hit(r, ray_t, rec);
        const new_max = if (hit_left) rec.t else ray_t.max;
        const hit_right = self.right.hit(r, Interval{ .min = ray_t.min, .max = new_max }, rec);

        return hit_left or hit_right;
    }

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};
