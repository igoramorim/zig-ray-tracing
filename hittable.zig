const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Vec3 = vec3.Vec3;
const Material = @import("material.zig").Material;
const Ray = @import("ray.zig").Ray;
const Interval = @import("interval.zig").Interval;
const Sphere = @import("sphere.zig").Sphere;

pub const Hittable = union(enum) {
    sphere: Sphere,

    pub fn hit(self: Hittable, r: Ray, rayT: Interval, rec: *HitRecord) bool {
        switch (self) {
            inline else => |obj| return obj.hit(r, rayT, rec),
        }
    }
};

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
