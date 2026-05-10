const std = @import("std");

const vec3 = @import("vec3.zig");
const f3 = vec3.f3;
const Vec3 = vec3.Vec3;
const Point3 = vec3.Point3;
const common = @import("common.zig");
const Color = @import("color.zig").Color;
const Ray = @import("ray.zig").Ray;
const HitRecord = @import("hittable.zig").HitRecord;
const texture = @import("texture.zig");
const Texture = texture.Texture;
const SolidColor = texture.SolidColor;

pub const Material = struct {
    ptr: *const anyopaque,
    scatter_fn: *const fn (ptr: *const anyopaque, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool,
    emitted_fn: *const fn (ptr: *const anyopaque, u: f64, v: f64, p: Point3) Color,

    pub fn scatter(self: Material, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        return self.scatter_fn(self.ptr, ray_in, rec, attenuation, scattered);
    }

    pub fn emitted(self: Material, u: f64, v: f64, p: Point3) Color {
        return self.emitted_fn(self.ptr, u, v, p);
    }
};

pub const Lambertian = struct {
    const Self = @This();

    tex: Texture,

    pub fn init(allocator: std.mem.Allocator, albedo: Color) !Lambertian {
        const tex_color = try allocator.create(SolidColor);
        tex_color.* = SolidColor.init(albedo);
        return Lambertian{ .tex = tex_color.texture() };
    }

    pub fn init_texture(tex: Texture) Lambertian {
        return Lambertian{ .tex = tex };
    }

    pub fn mat(self: *const Lambertian) Material {
        return Material{
            .ptr = self,
            .scatter_fn = scatter,
            .emitted_fn = emitted,
        };
    }

    pub fn scatter(ptr: *const anyopaque, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        var scatter_direction: Vec3 = rec.normal + vec3.rand_unit_vector();

        // catch degenerate scatter direction
        if (vec3.near_zero(scatter_direction)) {
            scatter_direction = rec.normal;
        }

        scattered.* = Ray{ .origin = rec.p, .direction = scatter_direction, .time = ray_in.time };
        attenuation.* = self.tex.value(rec.u, rec.v, rec.p);
        return true;
    }

    pub fn emitted(_: *const anyopaque, _: f64, _: f64, _: Point3) Color {
        return Color{ 0.0, 0.0, 0.0 };
    }
};

pub const Metal = struct {
    const Self = @This();

    albedo: Color,
    fuzz: f64,

    pub fn init(albedo: Color, fuzz: f64) Metal {
        return Metal{ .albedo = albedo, .fuzz = if (fuzz > 1.0) 1.0 else fuzz };
    }

    pub fn mat(self: *const Metal) Material {
        return Material{
            .ptr = self,
            .scatter_fn = scatter,
            .emitted_fn = emitted,
        };
    }

    pub fn scatter(ptr: *const anyopaque, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        var reflected = vec3.reflect(ray_in.direction, rec.normal);
        reflected = vec3.unit_vector(reflected) + (f3(self.fuzz) * vec3.rand_unit_vector());
        scattered.* = Ray{ .origin = rec.p, .direction = reflected, .time = ray_in.time };
        attenuation.* = self.albedo;
        return (vec3.dot(scattered.direction, rec.normal) > 0);
    }

    pub fn emitted(_: *const anyopaque, _: f64, _: f64, _: Point3) Color {
        return Color{ 0.0, 0.0, 0.0 };
    }
};

pub const Dielectric = struct {
    const Self = @This();

    // refractive index in vacuum or air, or the ratio of the material's refractive index over
    // the refractive index of the enclosing media
    refractionIndex: f64,

    pub fn mat(self: *const Dielectric) Material {
        return Material{
            .ptr = self,
            .scatter_fn = scatter,
            .emitted_fn = emitted,
        };
    }

    pub fn scatter(ptr: *const anyopaque, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        // attenuation 1 means the glass surface absorbs nothing
        attenuation.* = Color{ 1.0, 1.0, 1.0 };
        const ri = if (rec.front_face) (1.0 / self.refractionIndex) else self.refractionIndex;

        const unit_direction = vec3.unit_vector(ray_in.direction);
        const cos_theta = @min(vec3.dot(-unit_direction, rec.normal), 1.0);
        const sin_theta = @sqrt(1.0 - (cos_theta * cos_theta));

        const cannot_refract = (ri * sin_theta) > 1.0;
        var direction: Vec3 = undefined;

        if (cannot_refract or reflectance(cos_theta, ri) > common.rand_f64_01()) {
            direction = vec3.reflect(unit_direction, rec.normal);
        } else {
            direction = vec3.refract(unit_direction, rec.normal, ri);
        }

        scattered.* = Ray{ .origin = rec.p, .direction = direction, .time = ray_in.time };
        return true;
    }

    fn reflectance(cosine: f64, refraction_index: f64) f64 {
        // use Schlick's approximation for reflectance
        var r0 = (1 - refraction_index) / (1 + refraction_index);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std.math.pow(f64, (1 - cosine), 5);
    }

    pub fn emitted(_: *const anyopaque, _: f64, _: f64, _: Point3) Color {
        return Color{ 0.0, 0.0, 0.0 };
    }
};

pub const DiffuseLight = struct {
    const Self = @This();

    tex: Texture,

    pub fn init_tex(tex: Texture) DiffuseLight {
        return DiffuseLight{ .tex = tex };
    }

    pub fn init_color(allocator: std.mem.Allocator, emit: Color) !DiffuseLight {
        const tex = try allocator.create(SolidColor);
        tex.* = texture.SolidColor.init(emit);
        return DiffuseLight{ .tex = tex.texture() };
    }

    pub fn mat(self: *const DiffuseLight) Material {
        return Material{
            .ptr = self,
            .scatter_fn = scatter,
            .emitted_fn = emitted,
        };
    }

    pub fn emitted(ptr: *const anyopaque, u: f64, v: f64, p: Point3) Color {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.tex.value(u, v, p);
    }

    pub fn scatter(_: *const anyopaque, _: Ray, _: HitRecord, _: *Color, _: *Ray) bool {
        return false;
    }
};

pub const Isotropic = struct {
    const Self = @This();

    tex: Texture,

    pub fn init_color(albedo: Color) Isotropic {
        return Isotropic{
            .tex = SolidColor.init(albedo).texture(),
        };
    }

    pub fn init_tex(tex: Texture) Isotropic {
        return Isotropic{
            .tex = tex,
        };
    }

    pub fn mat(self: *const Isotropic) Material {
        return Material{
            .ptr = self,
            .scatter_fn = scatter,
            .emitted_fn = emitted,
        };
    }

    pub fn emitted(_: *const anyopaque, _: f64, _: f64, _: Point3) Color {
        return Color{ 0.0, 0.0, 0.0 };
    }

    pub fn scatter(ptr: *const anyopaque, ray_in: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        scattered.* = Ray{ .origin = rec.p, .direction = vec3.rand_unit_vector(), .time = ray_in.time };
        attenuation.* = self.tex.value(rec.u, rec.v, rec.p);

        return true;
    }
};
