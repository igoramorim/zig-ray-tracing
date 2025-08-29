const std = @import("std");
const common = @import("common.zig");

pub const Vec3 = struct {
    x: f64 = 0.0,
    y: f64 = 0.0,
    z: f64 = 0.0,

    pub fn length(self: Vec3) f64 {
        return std.math.sqrt(self.length_squared());
    }

    pub fn length_squared(self: Vec3) f64 {
        return (self.x * self.x) + (self.y * self.y) + (self.z * self.z);
    }

    pub fn div_i64(self: Vec3, i: i64) Vec3 {
        const value: f64 = 1 / @as(f64, @floatFromInt(i));
        return self.mult_f64(value);
    }

    pub fn div_f64(self: Vec3, f: f64) Vec3 {
        const value: f64 = 1 / f;
        return self.mult_f64(value);
    }

    pub fn mult(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x * v.x,
            .y = self.y * v.y,
            .z = self.z * v.z,
        };
    }

    pub fn mult_i64(self: Vec3, i: i64) Vec3 {
        return Vec3{
            .x = self.x * @as(f64, @floatFromInt(i)),
            .y = self.y * @as(f64, @floatFromInt(i)),
            .z = self.z * @as(f64, @floatFromInt(i)),
        };
    }

    pub fn mult_f64(self: Vec3, f: f64) Vec3 {
        return Vec3{
            .x = f * self.x,
            .y = f * self.y,
            .z = f * self.z,
        };
    }

    pub fn sub(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x - v.x,
            .y = self.y - v.y,
            .z = self.z - v.z,
        };
    }

    pub fn add(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x + v.x,
            .y = self.y + v.y,
            .z = self.z + v.z,
        };
    }

    pub fn add_f64(self: Vec3, f: f64) Vec3 {
        return Vec3{
            .x = self.x + f,
            .y = self.y + f,
            .z = self.z + f,
        };
    }

    pub fn reverse(self: Vec3) Vec3 {
        return Vec3{
            .x = -self.x,
            .y = -self.y,
            .z = -self.z,
        };
    }

    pub fn rand_01() Vec3 {
        return Vec3{
            .x = common.rand_f64_01(),
            .y = common.rand_f64_01(),
            .z = common.rand_f64_01(),
        };
    }

    pub fn rand(min: f64, max: f64) Vec3 {
        return Vec3{
            .x = common.rand_f64(min, max),
            .y = common.rand_f64(min, max),
            .z = common.rand_f64(min, max),
        };
    }

    // return true if the vector is close to zero in all dimensions
    pub fn near_zero(self: Vec3) bool {
        const s: f64 = 1e-8;
        return @abs(self.x) < s and @abs(self.y) < s and @abs(self.z) < s;
    }
};

pub const Point3 = Vec3;

pub fn dot(u: Vec3, v: Vec3) f64 {
    return u.x * v.x +
        u.y * v.y +
        u.z * v.z;
}

pub fn cross(u: Vec3, v: Vec3) Vec3 {
    return Vec3{
        .x = u.y * v.z - u.z * v.y,
        .y = u.z * v.x - u.x * v.z,
        .z = u.x * v.y - u.y * v.x,
    };
}

pub fn unit_vector(v: Vec3) Vec3 {
    return v.div_f64(v.length());
}

pub fn rand_in_unit_disk() Vec3 {
    while (true) {
        const p = Vec3{
            .x = common.rand_f64(-1, 1),
            .y = common.rand_f64(-1, 1),
        };

        if (p.length_squared() < 1) return p;
    }
}

pub fn rand_unit_vector() Vec3 {
    while (true) {
        const p = Vec3.rand(-1.0, 1.0);
        const lensq = p.length_squared();
        // NOTE: in the tutorial there is another check here
        if (1e-160 < lensq and lensq < 1.0) {
            return p.div_f64(std.math.sqrt(lensq));
        }
    }
}

fn randomOnHemisphere(normal: Vec3) Vec3 {
    const onUnitSphere = rand_unit_vector();
    if (dot(onUnitSphere, normal) > 0.0) { // in the same hemisphere as the normal
        return onUnitSphere;
    }
    return onUnitSphere.Reverse();
}

pub fn reflect(v: Vec3, n: Vec3) Vec3 {
    const vxn = dot(v, n);
    return v.sub(n.mult_f64(vxn).mult_f64(2));
}

pub fn refract(uv: Vec3, n: Vec3, etai_over_etat: f64) Vec3 {
    const cos_theta = @min(dot(uv.reverse(), n), 1.0);
    const r_out_perp = n.mult_f64(cos_theta).add(uv).mult_f64(etai_over_etat);
    const r_out_parallel = n.mult_f64(-@sqrt((@abs(1.0 - r_out_perp.length_squared()))));
    return r_out_perp.add(r_out_parallel);
}
