const std = @import("std");
const common = @import("common.zig");

pub const Vec3 = @Vector(3, f64);

pub inline fn f3(f: f64) Vec3 {
    return Vec3{ f, f, f };
}

pub inline fn length(v: Vec3) f64 {
    return std.math.sqrt(length_squared(v));
}

pub inline fn length_squared(v: Vec3) f64 {
    return (v[0] * v[0]) + (v[1] * v[1]) + (v[2] * v[2]);
}

pub inline fn rand_01() Vec3 {
    return Vec3{
        common.rand_f64_01(),
        common.rand_f64_01(),
        common.rand_f64_01(),
    };
}

pub inline fn rand(min: f64, max: f64) Vec3 {
    return Vec3{
        common.rand_f64(min, max),
        common.rand_f64(min, max),
        common.rand_f64(min, max),
    };
}

// return true if the vector is close to zero in all dimensions
pub inline fn near_zero(v: Vec3) bool {
    const s: f64 = 1e-8;
    return @abs(v[0]) < s and @abs(v[1]) < s and @abs(v[2]) < s;
}

pub const Point3 = Vec3;

pub inline fn dot(u: Vec3, v: Vec3) f64 {
    return @reduce(.Add, u * v);
}

pub inline fn cross(u: Vec3, v: Vec3) Vec3 {
    return Vec3{
        u[1] * v[2] - u[2] * v[1],
        u[2] * v[0] - u[0] * v[2],
        u[0] * v[1] - u[1] * v[0],
    };
}

pub inline fn unit_vector(v: Vec3) Vec3 {
    return v / f3(length(v));
}

pub inline fn rand_in_unit_disk() Vec3 {
    while (true) {
        const p = Vec3{
            common.rand_f64(-1, 1),
            common.rand_f64(-1, 1),
            0.0,
        };

        if (length_squared(p) < 1) return p;
    }
}

pub inline fn rand_unit_vector() Vec3 {
    while (true) {
        const p = rand(-1.0, 1.0);
        const lensq = length_squared(p);
        // NOTE: in the tutorial there is another check here
        if (1e-160 < lensq and lensq < 1.0) {
            return p / f3((@sqrt(lensq)));
        }
    }
}

inline fn randomOnHemisphere(normal: Vec3) Vec3 {
    const onUnitSphere = rand_unit_vector();
    if (dot(onUnitSphere, normal) > 0.0) { // in the same hemisphere as the normal
        return onUnitSphere;
    }
    return -onUnitSphere;
}

pub inline fn reflect(v: Vec3, n: Vec3) Vec3 {
    return v - f3(2.0) * f3(dot(v, n)) * n;
}

pub inline fn refract(uv: Vec3, n: Vec3, etai_over_etat: f64) Vec3 {
    const cos_theta = @min(dot(-uv, n), 1.0);
    const r_out_perp: Vec3 = f3(etai_over_etat) * (uv + f3(cos_theta) * n);
    const r_out_parallel: Vec3 = f3(-@sqrt(@abs(1.0 - length_squared(r_out_perp)))) * n;
    return r_out_perp + r_out_parallel;
}
