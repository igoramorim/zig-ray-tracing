const std = @import("std");

const vec3 = @import("vec3.zig");
const Vec3 = vec3.Vec3;
const common = @import("common.zig");
const Color = @import("color.zig").Color;
const Ray = @import("ray.zig").Ray;
const HitRecord = @import("hittable.zig").HitRecord;

pub const Material = union(enum) {
    lamertian: Lambertian,
    metal: Metal,
    dielectric: Dielectric,

    pub fn scatter(self: Material, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        switch (self) {
            inline else => |mat| return mat.scatter(ray_in, rec, attenuation, scattered),
        }
    }
};

pub const Lambertian = struct {
    albedo: Color,

    pub fn init(albedo: Color) Lambertian {
        return Lambertian{ .albedo = albedo };
    }

    pub fn scatter(self: Lambertian, _: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        var scatter_direction = rec.normal.add(vec3.rand_unit_vector());

        // catch degenerate scatter direction
        if (scatter_direction.near_zero()) {
            scatter_direction = rec.normal;
        }

        scattered.* = Ray{ .origin = rec.p, .direction = scatter_direction };
        attenuation.* = self.albedo;
        return true;
    }
};

pub const Metal = struct {
    albedo: Color,
    fuzz: f64,

    pub fn init(albedo: Color, fuzz: f64) Metal {
        return Metal{ .albedo = albedo, .fuzz = if (fuzz > 1.0) 1.0 else fuzz };
    }

    pub fn scatter(self: Metal, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        var reflected = vec3.reflect(ray_in.direction, rec.normal);
        reflected = vec3.rand_unit_vector().mult_f64(self.fuzz).add(vec3.unit_vector(reflected));
        scattered.* = Ray{ .origin = rec.p, .direction = reflected };
        attenuation.* = self.albedo;
        return (vec3.dot(scattered.direction, rec.normal) > 0);
    }
};

pub const Dielectric = struct {
    // refractive index in vacuum or air, or the ratio of the material's refractive index over
    // the refractive index of the enclosing media
    refractionIndex: f64,

    pub fn scatter(self: Dielectric, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        // attenuation 1 means the glass surface absorbs nothing
        attenuation.* = Color{ .x = 1.0, .y = 1.0, .z = 1.0 };
        const ri = if (rec.front_face) (1.0 / self.refractionIndex) else self.refractionIndex;

        const unit_direction = vec3.unit_vector(ray_in.direction);
        const cos_theta = @min(vec3.dot(unit_direction.reverse(), rec.normal), 1.0);
        const sin_theta = @sqrt(1.0 - (cos_theta * cos_theta));

        const cannot_refract = (ri * sin_theta) > 1.0;
        var direction: Vec3 = undefined;

        if (cannot_refract or reflectance(cos_theta, ri) > common.rand_f64_01()) {
            direction = vec3.reflect(unit_direction, rec.normal);
        } else {
            direction = vec3.refract(unit_direction, rec.normal, ri);
        }

        scattered.* = Ray{ .origin = rec.p, .direction = direction };
        return true;
    }

    fn reflectance(cosine: f64, refraction_index: f64) f64 {
        // use Schlick's approximation for reflectance
        var r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std.math.pow(f64, (1 - cosine), 5);
    }
};
