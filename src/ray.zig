const Point3 = @import("vec3.zig").Point3;
const vec3 = @import("vec3.zig");
const f3 = vec3.f3;
const Vec3 = vec3.Vec3;

pub const Ray = struct {
    origin: Point3,
    direction: Vec3,
    time: f64 = 0,

    pub fn at(self: Ray, t: f64) Point3 {
        return self.origin + f3(t) * self.direction;
    }
};
