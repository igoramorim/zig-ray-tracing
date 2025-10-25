const std = @import("std");
const stdout = std.io.getStdOut().writer();
const debug = std.debug;

const vec3 = @import("vec3.zig");
const Vec3 = vec3.Vec3;
const Point3 = vec3.Point3;
const common = @import("common.zig");
const color = @import("color.zig");
const Color = color.Color;
const interval = @import("interval.zig");
const Interval = interval.Interval;
const Hittable = @import("hittable.zig").Hittable;
const Sphere = @import("sphere.zig").Sphere;
const material = @import("material.zig");
const Material = material.Material;
const Lambertian = material.Lambertian;
const Metal = material.Metal;
const Dielectric = material.Dielectric;
const Ray = @import("ray.zig").Ray;
const HitRecord = @import("hittable.zig").HitRecord;
const HittableList = @import("hittable_list.zig").HittableList;
const Camera = @import("camera.zig").Camera;

pub fn main() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const allocator = gpa.allocator();
    defer _ = gpa.deinit();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    const mat_ground = Material{ .lamertian = Lambertian.init(Color{ .x = 0.5, .y = 0.5, .z = 0.5 }) };
    const ground = Sphere.init(Point3{ .x = 0.0, .y = -1000.0, .z = 0.0 }, 1000.0, mat_ground);
    try world.add(Hittable{ .sphere = ground });

    const it: i64 = 11;
    var a: i64 = -it;
    while (a < it) : (a = a + 1) {
        var b: i64 = -it;
        while (b < it) : (b = b + 1) {
            const choose_mat = common.rand_f64_01();

            const center = Point3{
                .x = @as(f64, @floatFromInt(a)) + (0.9 * common.rand_f64_01()),
                .y = 0.2,
                .z = @as(f64, @floatFromInt(b)) + (0.9 * common.rand_f64_01()),
            };

            if (center.sub(Point3{ .x = 4.0, .y = 0.2, .z = 0.0 }).length() > 0.9) {
                if (choose_mat < 0.8) {
                    // diffuse
                    const albedo = Color.rand_01().mult(Color.rand_01());
                    const mat = Material{ .lamertian = Lambertian.init(albedo) };
                    const center2 = center.add(Vec3{ .y = common.rand_f64(0.0, 0.5) });
                    const sphere = Sphere.init_moving(center, center2, 0.2, mat);
                    try world.add(Hittable{ .sphere = sphere });
                } else if (choose_mat < 0.95) {
                    // metal
                    const albedo = Color.rand(0.5, 1.0);
                    const fuzz = common.rand_f64(0.0, 0.5);
                    const mat = Metal.init(albedo, fuzz);
                    const sphere = Sphere.init(center, 0.2, Material{ .metal = mat });
                    try world.add(Hittable{ .sphere = sphere });
                } else {
                    // glass
                    const mat = Dielectric{ .refractionIndex = 1.5 };
                    const sphere = Sphere.init(center, 0.2, Material{ .dielectric = mat });
                    try world.add(Hittable{ .sphere = sphere });
                }
            }
        }
    }

    const mat_glass = Material{ .dielectric = Dielectric{ .refractionIndex = 1.5 } };
    const sphere_glass = Sphere.init(Point3{ .x = 0.0, .y = 1.0, .z = 0.0 }, 1.0, mat_glass);
    try world.add(Hittable{ .sphere = sphere_glass });

    const mat_diffuse = Material{ .lamertian = Lambertian.init(Color{ .x = 0.4, .y = 0.2, .z = 0.1 }) };
    const sphere_diffuse = Sphere.init(Point3{ .x = -4.0, .y = 1.0, .z = 0.0 }, 1.0, mat_diffuse);
    try world.add(Hittable{ .sphere = sphere_diffuse });

    const mat_metal = Material{ .metal = Metal.init(Color{ .x = 0.7, .y = 0.6, .z = 0.5 }, 0.0) };
    const sphere_metal = Sphere.init(Point3{ .x = 4.0, .y = 1.0, .z = 0.0 }, 1.0, mat_metal);
    try world.add(Hittable{ .sphere = sphere_metal });

    // camera
    var cam = Camera{};
    cam.aspect_radio = 16.0 / 9.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.vfov = 20;
    cam.look_from = Point3{ .x = 13.0, .y = 2, .z = 3 };
    cam.look_at = Point3{};
    cam.vup = Vec3{ .y = 1 };
    cam.defocus_angle = 0.6;
    cam.focus_dist = 10.0;

    try cam.render(allocator, world);
}
