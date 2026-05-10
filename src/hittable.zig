const std = @import("std");
const math = std.math;

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Vec3 = vec3.Vec3;
const f3 = vec3.f3;
const Material = @import("material.zig").Material;
const Ray = @import("ray.zig").Ray;
const Interval = @import("interval.zig").Interval;
const common = @import("common.zig");
const Texture = @import("texture.zig").Texture;
const Color = @import("color.zig").Color;
const Isotropic = @import("material.zig").Isotropic;
const SolidColor = @import("texture.zig").SolidColor;

pub const HitRecord = struct {
    p: Point3 = Point3{ 0.0, 0.0, 0.0 },
    normal: Vec3 = Vec3{ 0.0, 0.0, 0.0 },
    mat: Material = undefined,
    t: f64 = undefined,
    front_face: bool = undefined,
    u: f64 = undefined,
    v: f64 = undefined,

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

    /// p: a given point on the sphere of radius one, centered at the origin
    /// u: returned value [0,1] of angle around the Y axis from X=-1
    /// v: returned value [0,1] of angle from Y=-1 to Y=+1
    ///
    /// < 1 0 0 > yields < 0.50 0.50 >   < -1  0  0 > yields <0.00 0.50>
    /// < 0 1 0 > yields < 0.50 1.00 >   <  0 -1  0 > yields <0.50 0.00>
    /// < 0 0 1 > yields < 0.25 0.50 >   <  0  0 -1 > yields <0.75 0.50>
    fn get_uv(_: Sphere, p: Point3, u: *f64, v: *f64) void {
        const theta = math.acos(-p[1]);
        const phi = math.atan2(-p[2], p[0]) + math.pi;

        u.* = phi / (2 * math.pi);
        v.* = theta / math.pi;
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
        self.get_uv(outward_normal, &rec.u, &rec.v);
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

        var aabb = AABB{ .x = x, .y = y, .z = z };
        aabb.pad_to_minimums();

        return aabb;
    }

    pub fn init_from_boxes(box0: AABB, box1: AABB) AABB {
        var aabb = AABB{
            .x = Interval.init_from_intervals(box0.x, box1.x),
            .y = Interval.init_from_intervals(box0.y, box1.y),
            .z = Interval.init_from_intervals(box0.z, box1.z),
        };
        aabb.pad_to_minimums();
        return aabb;
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

    fn pad_to_minimums(self: *AABB) void {
        const delta: f32 = 0.0001;
        if (self.x.size() < delta) self.x = self.x.expand(delta);
        if (self.y.size() < delta) self.y = self.y.expand(delta);
        if (self.z.size() < delta) self.z = self.z.expand(delta);
    }

    pub fn add_vec3(self: AABB, offset: Vec3) AABB {
        const x = self.x.add(offset[0]);
        const y = self.x.add(offset[1]);
        const z = self.x.add(offset[2]);

        var aabb = AABB{ .x = x, .y = y, .z = z };
        aabb.pad_to_minimums();

        return aabb;
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

pub const Quad = struct {
    const Self = @This();

    q: Point3 = undefined,
    u: Vec3 = undefined,
    v: Vec3 = undefined,
    w: Vec3 = undefined,
    mat: Material = undefined,
    bbox: AABB = undefined,
    normal: Vec3 = undefined,
    d: f64 = undefined,

    pub fn init(q: Point3, u: Vec3, v: Vec3, mat: Material) Quad {
        var quad = Quad{
            .q = q,
            .u = u,
            .v = v,
            .mat = mat,
        };

        const n = vec3.cross(quad.u, quad.v);
        quad.normal = vec3.unit_vector(n);
        quad.d = vec3.dot(quad.normal, quad.q);
        quad.w = n / f3(vec3.dot(n, n));

        quad.set_bounding_box();

        return quad;
    }

    fn set_bounding_box(self: *Quad) void {
        const bbox_diagonal1 = AABB.init_from_points(self.q, self.q + self.u + self.v);
        const bbox_diagonal2 = AABB.init_from_points(self.q + self.u, self.q + self.v);
        self.bbox = AABB.init_from_boxes(bbox_diagonal1, bbox_diagonal2);
    }

    pub fn hittable(self: *const Quad) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        const denom = vec3.dot(self.normal, r.direction);

        // no hit if the ray is parallel to the plane
        if (@abs(denom) < 1e-8) return false;

        // return false if the hit point parameter t is outside the ray interval
        const t: f64 = (self.d - vec3.dot(self.normal, r.origin)) / denom;
        if (!ray_t.contains(t)) return false;

        // determine if the hit point lies within the planar shape using its plane coordinates
        const intersection = r.at(t);
        const planar_hitp_vector = intersection - self.q;
        const alpha = vec3.dot(self.w, vec3.cross(planar_hitp_vector, self.v));
        const beta = vec3.dot(self.w, vec3.cross(self.u, planar_hitp_vector));

        if (!is_interior(alpha, beta, rec)) return false;

        // ray hits the 2D shape
        rec.t = t;
        rec.p = intersection;
        rec.mat = self.mat;
        rec.set_face_normal(r, self.normal);

        return true;
    }

    fn is_interior(a: f64, b: f64, rec: *HitRecord) bool {
        const unit_interval = Interval{ .min = 0, .max = 1 };

        if (!unit_interval.contains(a) or !unit_interval.contains(b)) return false;

        rec.u = a;
        rec.v = b;

        return true;
    }

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};

pub const Box = struct {
    const Self = @This();

    sides: [6]Quad,
    bbox: AABB,

    pub fn init(a: Point3, b: Point3, mat: Material) Box {
        const min = Point3{
            @min(a[0], b[0]),
            @min(a[1], b[1]),
            @min(a[2], b[2]),
        };

        const max = Point3{
            @max(a[0], b[0]),
            @max(a[1], b[1]),
            @max(a[2], b[2]),
        };

        const dx = Vec3{ max[0] - min[0], 0.0, 0.0 };
        const dy = Vec3{ 0.0, max[1] - min[1], 0.0 };
        const dz = Vec3{ 0.0, 0.0, max[2] - min[2] };

        return Box{
            .sides = .{
                Quad.init(Point3{ min[0], min[1], max[2] }, dx, dy, mat), // front
                Quad.init(Point3{ max[0], min[1], max[2] }, -dz, dy, mat), //right
                Quad.init(Point3{ max[0], min[1], min[2] }, -dx, dy, mat), // back
                Quad.init(Point3{ min[0], min[1], min[2] }, dz, dy, mat), // left
                Quad.init(Point3{ min[0], max[1], max[2] }, dx, -dz, mat), // top
                Quad.init(Point3{ min[0], min[1], min[2] }, dx, dz, mat), // bottom
            },
            .bbox = AABB.init_from_points(min, max),
        };
    }

    pub fn hittable(self: *const Box) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        var hit_anything = false;
        var closes_so_far = ray_t.max;

        for (&self.sides) |*side| {
            if (side.hittable().hit(r, Interval{ .min = ray_t.min, .max = closes_so_far }, rec)) {
                hit_anything = true;
                closes_so_far = rec.t;
            }
        }

        return hit_anything;
    }

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};

pub const Translate = struct {
    const Self = @This();

    object: Hittable,
    offset: Vec3,
    bbox: AABB,

    pub fn init(object: Hittable, offset: Vec3) Translate {
        return Translate{
            .object = object,
            .offset = offset,
            .bbox = object.bounding_box().add_vec3(offset),
        };
    }

    pub fn hittable(self: *const Translate) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        // move the ray backwards by the offset
        const offset_ray = Ray{
            .origin = r.origin - self.offset,
            .direction = r.direction,
            .time = r.time,
        };

        // determine whether an intersection exists along the offset ray (and if so, where)
        if (!self.object.hit(offset_ray, ray_t, rec)) return false;

        // move the intersection point forwards by the offset
        rec.p += self.offset;

        return true;
    }

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};

pub const RotateY = struct {
    const Self = @This();

    object: Hittable,
    sin_theta: f64,
    cos_theta: f64,
    bbox: AABB,

    pub fn init(object: Hittable, angle: f64) RotateY {
        const radians = common.deg_to_rad(angle);
        const sin_theta = @sin(radians);
        const cos_theta = @cos(radians);
        const bbox = object.bounding_box();

        var min = Point3{ common.infinity, common.infinity, common.infinity };
        var max = Point3{ -common.infinity, -common.infinity, -common.infinity };

        for (0..2) |ui| {
            for (0..2) |uj| {
                for (0..2) |uk| {
                    const i: f64 = @floatFromInt(ui);
                    const j: f64 = @floatFromInt(uj);
                    const k: f64 = @floatFromInt(uk);

                    const x: f64 = i * bbox.x.max + (1 - i) * bbox.x.min;
                    const y: f64 = j * bbox.y.max + (1 - j) * bbox.y.min;
                    const z: f64 = k * bbox.z.max + (1 - k) * bbox.z.min;

                    const newx = cos_theta * x + sin_theta * z;
                    const newz = -sin_theta * x + cos_theta * z;

                    const tester = Vec3{ newx, y, newz };

                    min = Point3{
                        @min(min[0], tester[0]),
                        @min(min[1], tester[1]),
                        @min(min[2], tester[2]),
                    };

                    max = Point3{
                        @max(max[0], tester[0]),
                        @max(max[1], tester[1]),
                        @max(max[2], tester[2]),
                    };
                }
            }
        }

        return RotateY{
            .object = object,
            .sin_theta = sin_theta,
            .cos_theta = cos_theta,
            .bbox = AABB.init_from_points(min, max),
        };
    }

    pub fn hittable(self: *const RotateY) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        // transform the ray from world space to object space
        const origin = Point3{
            (self.cos_theta * r.origin[0]) - (self.sin_theta * r.origin[2]),
            r.origin[1],
            (self.sin_theta * r.origin[0]) + (self.cos_theta * r.origin[2]),
        };

        const direction = Vec3{
            (self.cos_theta * r.direction[0]) - (self.sin_theta * r.direction[2]),
            r.direction[1],
            (self.sin_theta * r.direction[0]) + (self.cos_theta * r.direction[2]),
        };

        const rotated_ray = Ray{
            .origin = origin,
            .direction = direction,
            .time = r.time,
        };

        // determine whether an intersection exists in object space (and if so, where)
        if (!self.object.hit(rotated_ray, ray_t, rec)) return false;

        // transform the intersection from object space back to world space
        rec.p = Point3{
            (self.cos_theta * rec.p[0]) + (self.sin_theta * rec.p[2]),
            rec.p[1],
            (-self.sin_theta * rec.p[0]) + (self.cos_theta * rec.p[2]),
        };

        rec.normal = Vec3{
            (self.cos_theta * rec.normal[0]) + (self.sin_theta * rec.normal[2]),
            rec.normal[1],
            (-self.sin_theta * rec.normal[0]) + (self.cos_theta * rec.normal[2]),
        };

        return true;
    }

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.bbox;
    }
};

pub const ConstantMedium = struct {
    const Self = @This();

    boundary: Hittable,
    neg_inv_density: f64,
    phase_function: Material,

    pub fn init_texture(allocator: std.mem.Allocator, boundary: Hittable, density: f64, tex: Texture) !ConstantMedium {
        const iso = try allocator.create(Isotropic);
        iso.* = Isotropic.init_tex(tex);

        return ConstantMedium{
            .boundary = boundary,
            .neg_inv_density = @as(f64, -1.0) / density,
            .phase_function = iso.mat(),
        };
    }

    pub fn init_color(allocator: std.mem.Allocator, boundary: Hittable, density: f64, albedo: Color) !ConstantMedium {
        const tex = try allocator.create(SolidColor);
        tex.* = SolidColor.init(albedo);

        const iso = try allocator.create(Isotropic);
        iso.* = Isotropic.init_tex(tex.texture());

        return ConstantMedium{
            .boundary = boundary,
            .neg_inv_density = @as(f64, -1.0) / density,
            .phase_function = iso.mat(),
        };
    }

    pub fn hittable(self: *const ConstantMedium) Hittable {
        return Hittable{
            .ptr = self,
            .hit_fn = hit,
            .bounding_box_fn = bounding_box,
        };
    }

    pub fn hit(ptr: *const anyopaque, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        var rec1 = HitRecord{};
        var rec2 = HitRecord{};

        if (!self.boundary.hit(r, Interval.universe(), &rec1)) return false;

        if (!self.boundary.hit(r, Interval{ .min = rec1.t + 0.0001, .max = common.infinity }, &rec2)) return false;

        if (rec1.t < ray_t.min) rec1.t = ray_t.min;
        if (rec2.t > ray_t.max) rec2.t = ray_t.max;

        if (rec1.t >= rec2.t) return false;

        if (rec1.t < 0) rec1.t = 0;

        const ray_length = vec3.length(r.direction);
        const distance_inside_boundary = (rec2.t - rec1.t) * ray_length;
        const hit_distance = self.neg_inv_density * @log(common.rand_f64_01());

        if (hit_distance > distance_inside_boundary) return false;

        rec.t = rec1.t + hit_distance / ray_length;
        rec.p = r.at(rec.t);
        rec.normal = Vec3{ 1.0, 0.0, 0.0 };
        rec.front_face = true;
        rec.mat = self.phase_function;

        return true;
    }

    pub fn bounding_box(ptr: *const anyopaque) AABB {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.boundary.bounding_box();
    }
};
