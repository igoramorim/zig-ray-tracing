const std = @import("std");
const stderr = std.io.getStdErr().writer();

const vec3 = @import("vec3.zig");
const Vec3 = vec3.Vec3;
const Point3 = vec3.Point3;
const common = @import("common.zig");
const color = @import("color.zig");
const Color = color.Color;
const interval = @import("interval.zig");
const Interval = interval.Interval;
const Hittable = @import("hittable.zig").Hittable;
const Sphere = @import("hittable.zig").Sphere;
const material = @import("material.zig");
const Lambertian = material.Lambertian;
const Metal = material.Metal;
const Dielectric = material.Dielectric;
const Ray = @import("ray.zig").Ray;
const HitRecord = @import("hittable.zig").HitRecord;
const HittableList = @import("hittable.zig").HittableList;
const Camera = @import("camera.zig").Camera;
const BVHNode = @import("hittable.zig").BVHNode;
const texture = @import("texture.zig");
const Checker = texture.Checker;

pub fn main() !void {
    switch (2) {
        1 => try bouncing_spheres(),
        2 => try checkered_spheres(),
        else => stderr.print("invalid scene\n", .{}),
    }
}

fn bouncing_spheres() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    const tex_checker = try allocator.create(Checker);
    tex_checker.* = try Checker.init_color(allocator, 0.32, vec3.rand_01() * vec3.rand_01(), Color{ 0.9, 0.9, 0.9 });
    const mat_ground = Lambertian.init_texture(tex_checker.texture());
    const ground = Sphere.init(Point3{ 0.0, -1000.0, 0.0 }, 1000.0, mat_ground.mat());
    try world.add(ground.hittable());

    const it: i32 = 11;
    var a: i32 = -it;
    while (a < it) : (a = a + 1) {
        var b: i32 = -it;
        while (b < it) : (b = b + 1) {
            const choose_mat = common.rand_f64_01();

            const center = Point3{
                @as(f64, @floatFromInt(a)) + (0.9 * common.rand_f64_01()),
                0.2,
                @as(f64, @floatFromInt(b)) + (0.9 * common.rand_f64_01()),
            };

            if (vec3.length(center - Point3{ 4.0, 0.2, 0.0 }) > 0.9) {
                if (choose_mat < 0.8) {
                    // diffuse
                    const albedo = vec3.rand_01() * vec3.rand_01();
                    const mat = try allocator.create(Lambertian);
                    mat.* = try Lambertian.init(allocator, albedo);
                    const center2 = center + Vec3{ 0.0, common.rand_f64(0.0, 0.5), 0.0 };
                    const sphere = try allocator.create(Sphere);
                    sphere.* = Sphere.init_moving(center, center2, 0.2, mat.mat());
                    try world.add(sphere.hittable());
                } else if (choose_mat < 0.95) {
                    // metal
                    const albedo = vec3.rand(0.5, 1.0);
                    const fuzz = common.rand_f64(0.0, 0.5);
                    const mat = try allocator.create(Metal);
                    mat.* = Metal.init(albedo, fuzz);
                    const sphere = try allocator.create(Sphere);
                    sphere.* = Sphere.init(center, 0.2, mat.mat());
                    try world.add(sphere.hittable());
                } else {
                    // glass
                    const mat = try allocator.create(Dielectric);
                    mat.* = Dielectric{ .refractionIndex = 1.5 };
                    const sphere = try allocator.create(Sphere);
                    sphere.* = Sphere.init(center, 0.2, mat.mat());
                    try world.add(sphere.hittable());
                }
            }
        }
    }

    const mat_glass = Dielectric{ .refractionIndex = 1.5 };
    const sphere_glass = Sphere.init(Point3{ 0.0, 1.0, 0.0 }, 1.0, mat_glass.mat());
    try world.add(sphere_glass.hittable());

    const mat_diffuse = try Lambertian.init(allocator, Color{ 0.4, 0.2, 0.1 });
    const sphere_diffuse = Sphere.init(Point3{ -4.0, 1.0, 0.0 }, 1.0, mat_diffuse.mat());
    try world.add(sphere_diffuse.hittable());

    const mat_metal = Metal.init(Color{ 0.7, 0.6, 0.5 }, 0.0);
    const sphere_metal = Sphere.init(Point3{ 4.0, 1.0, 0.0 }, 1.0, mat_metal.mat());
    try world.add(sphere_metal.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 16.0 / 9.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.vfov = 20;
    cam.look_from = Point3{ 13.0, 2.0, 3.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.6;
    cam.focus_dist = 10.0;

    try cam.render(bvh.hittable());
}

fn checkered_spheres() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    const tex_checker = try allocator.create(Checker);
    tex_checker.* = try Checker.init_color(allocator, 0.32, vec3.rand_01() * vec3.rand_01(), Color{ 0.9, 0.9, 0.9 });
    const mat = Lambertian.init_texture(tex_checker.texture());

    const sphere1 = Sphere.init(Point3{ 0.0, -10.0, 0.0 }, 10.0, mat.mat());
    try world.add(sphere1.hittable());

    const sphere2 = Sphere.init(Point3{ 0.0, 10.0, 0.0 }, 10.0, mat.mat());
    try world.add(sphere2.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 16.0 / 9.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.vfov = 20;
    cam.look_from = Point3{ 13.0, 2.0, 3.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;

    try cam.render(bvh.hittable());
}
