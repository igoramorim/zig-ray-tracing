const Point3 = @import("vec3.zig").Point3;
const Vec3 = @import("vec3.zig").Vec3;

pub const Ray = struct {
    origin: Point3,
    direction: Vec3,
    time: f64 = 0,

    pub fn at(self: Ray, t: f64) Point3 {
        return self.direction.mult_f64(t).add(self.origin);
    }
};
