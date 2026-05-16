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
const Image = texture.Image;
const flags = @import("flags");
const Noise = texture.Noise;
const Quad = @import("hittable.zig").Quad;
const DiffuseLight = @import("material.zig").DiffuseLight;
const Box = @import("hittable.zig").Box;
const RotateY = @import("hittable.zig").RotateY;
const Translate = @import("hittable.zig").Translate;
const ConstantMedium = @import("hittable.zig").ConstantMedium;

// TODO:
// - (zig) Remove the hittable() material() methods. Add an attr that holds it and it is initialized in init()
// - (zig) Parallelize
// - Add other types of lights (directional and spot)
// - Cast shadows
// - Scale transformation
// - Rotate in X and Z axis

pub fn main() !void {
    var args = std.process.args();

    const scene_id = flags.parse(u8, &args, "scene") catch |err| {
        try stderr.print("{any}: scene\n{s}\n", .{ err, help_msg });
        std.process.exit(1);
    };

    switch (scene_id) {
        1 => try bouncing_spheres(),
        2 => try checkered_spheres(),
        3 => try earth(),
        4 => try perlin_spheres(),
        5 => try quads(),
        6 => try diffuse_light(),
        7 => try empty_cornell_box(),
        8 => try cornell_box(),
        9 => try cornell_box_smoke(),
        10 => try all_features(),
        else => try stderr.print("invalid scene\n{s}\n", .{help_msg}),
    }
}

const help_msg =
    \\NAME
    \\  zrt - renders a scene using Ray Tracing and outputs it in ppm3 format
    \\
    \\USAGE
    \\  zrt [FLAGS] > image.ppm
    \\
    \\FLAGS
    \\  -scene [id] - specifies wich scene to render
    \\
    \\Avaiable scenes:
    \\  1 Bouncing Spheres
    \\  2 Checkered Spheres
    \\  3 Earth (texture)
    \\  4 Perlin Spheres
    \\  5 Quads
    \\  6 Diffuse Light
    \\  7 Empty Cornell Box
    \\  8 Cornell Box
    \\  9 Cornell Box with Smoke
    \\ 10 All features
;

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

fn earth() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    const tex_earth = try allocator.create(Image);
    tex_earth.* = try Image.init(allocator, "assets/textures/earth.jpg");
    const mat = Lambertian.init_texture(tex_earth.texture());

    const globe = Sphere.init(Point3{ 0.0, 0.0, 0.0 }, 2.0, mat.mat());
    try world.add(globe.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 16.0 / 9.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.vfov = 20;
    cam.look_from = Point3{ 0.0, 0.0, 12.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;

    try cam.render(bvh.hittable());
}

fn perlin_spheres() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    const tex_perlin = Noise.init(4.0);
    const mat = Lambertian.init_texture(tex_perlin.texture());

    const sphere1 = Sphere.init(Point3{ 0.0, -1000.0, 0.0 }, 1000.0, mat.mat());
    const sphere2 = Sphere.init(Point3{ 0.0, 2.0, 0.0 }, 2.0, mat.mat());
    try world.add(sphere1.hittable());
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

fn quads() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    // materials
    const red = try Lambertian.init(allocator, Color{ 1.0, 0.2, 0.2 });
    const green = try Lambertian.init(allocator, Color{ 0.2, 1.0, 0.2 });
    const blue = try Lambertian.init(allocator, Color{ 0.2, 0.2, 1.0 });
    const orange = try Lambertian.init(allocator, Color{ 1.0, 0.5, 0.0 });
    const teal = try Lambertian.init(allocator, Color{ 0.2, 0.8, 0.8 });

    // quads
    const left = Quad.init(Point3{ -3.0, -2.0, 5.0 }, Vec3{ 0.0, 0.0, -4.0 }, Vec3{ 0.0, 4.0, 0.0 }, red.mat());
    try world.add(left.hittable());

    const back = Quad.init(Point3{ -2.0, -2.0, 0.0 }, Vec3{ 4.0, 0.0, 0.0 }, Vec3{ 0.0, 4.0, 0.0 }, green.mat());
    try world.add(back.hittable());

    const right = Quad.init(Point3{ 3.0, -2.0, 1.0 }, Vec3{ 0.0, 0.0, 4.0 }, Vec3{ 0.0, 4.0, 0.0 }, blue.mat());
    try world.add(right.hittable());

    const upper = Quad.init(Point3{ -2.0, 3.0, 1.0 }, Vec3{ 4.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, 4.0 }, orange.mat());
    try world.add(upper.hittable());

    const lower = Quad.init(Point3{ -2.0, -3.0, 5.0 }, Vec3{ 4.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, -4.0 }, teal.mat());
    try world.add(lower.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 1.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.vfov = 80;
    cam.look_from = Point3{ 0.0, 0.0, 9.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;

    try cam.render(bvh.hittable());
}

fn diffuse_light() !void {
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
    tex_checker.* = try Checker.init_color(allocator, 2.0, vec3.rand_01() * vec3.rand_01(), Color{ 0.9, 0.9, 0.9 });
    const mat_checker = Lambertian.init_texture(tex_checker.texture());
    const ground = Sphere.init(Point3{ 0.0, -1000.0, 0.0 }, 1000.0, mat_checker.mat());
    try world.add(ground.hittable());

    const tex_perlin = Noise.init(4.0);
    const mat_pertlin = Lambertian.init_texture(tex_perlin.texture());
    const sphere = Sphere.init(Point3{ 0.0, 2.0, 0.0 }, 2.0, mat_pertlin.mat());
    try world.add(sphere.hittable());

    const diff_light_red = try DiffuseLight.init_color(allocator, Color{ 4.0, 1.0, 1.0 });
    const light_red = Quad.init(Point3{ 3.0, 1.0, -2.0 }, Vec3{ 2.0, 0.0, 0.0 }, Vec3{ 0.0, 2.0, 0.0 }, diff_light_red.mat());
    try world.add(light_red.hittable());

    const diff_light_green = try DiffuseLight.init_color(allocator, Color{ 1.0, 4.0, 1.0 });
    const light_green = Quad.init(Point3{ 3.0, 1.0, 3.0 }, Vec3{ 2.0, 0.0, 0.0 }, Vec3{ 0.0, 2.0, 0.0 }, diff_light_green.mat());
    try world.add(light_green.hittable());

    const diff_light_blue = try DiffuseLight.init_color(allocator, Color{ 1.0, 1.0, 4.0 });
    const light_blue = Sphere.init(Point3{ 0.0, 7.0, 0.0 }, 2.0, diff_light_blue.mat());
    try world.add(light_blue.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 16.0 / 9.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 100;
    cam.max_depth = 50;
    cam.vfov = 20;
    cam.look_from = Point3{ 20.0, 3.0, 6.0 };
    cam.look_at = Point3{ 0.0, 2.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(bvh.hittable());
}

fn empty_cornell_box() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    const red = try Lambertian.init(allocator, Color{ 0.65, 0.05, 0.05 });
    const white = try Lambertian.init(allocator, Color{ 0.73, 0.73, 0.73 });
    const green = try Lambertian.init(allocator, Color{ 0.12, 0.45, 0.15 });
    const light_mat = try DiffuseLight.init_color(allocator, Color{ 15.0, 15.0, 15.0 });

    const left = Quad.init(Point3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, green.mat());
    try world.add(left.hittable());

    const right = Quad.init(Point3{ 0.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, red.mat());
    try world.add(right.hittable());

    const light = Quad.init(Point3{ 343.0, 554.0, 332.0 }, Vec3{ -130.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, -105.0 }, light_mat.mat());
    try world.add(light.hittable());

    const center = Quad.init(Point3{ 0.0, 0.0, 0.0 }, Vec3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, white.mat());
    try world.add(center.hittable());

    const top = Quad.init(Point3{ 555.0, 555.0, 555.0 }, Vec3{ -555.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, -555.0 }, white.mat());
    try world.add(top.hittable());

    const bottom = Quad.init(Point3{ 0.0, 0.0, 555.0 }, Vec3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, white.mat());
    try world.add(bottom.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 1.0;
    cam.image_width = 600;
    cam.samples_per_pixel = 200;
    cam.max_depth = 50;
    cam.vfov = 40;
    cam.look_from = Point3{ 278.0, 278.0, -800.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(bvh.hittable());
}

fn cornell_box() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    // materials
    const red = try Lambertian.init(allocator, Color{ 0.65, 0.05, 0.05 });
    const white = try Lambertian.init(allocator, Color{ 0.73, 0.73, 0.73 });
    const green = try Lambertian.init(allocator, Color{ 0.12, 0.45, 0.15 });
    const light_mat = try DiffuseLight.init_color(allocator, Color{ 15.0, 15.0, 15.0 });

    // walls
    const left = Quad.init(Point3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, green.mat());
    try world.add(left.hittable());

    const right = Quad.init(Point3{ 0.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, red.mat());
    try world.add(right.hittable());

    const light = Quad.init(Point3{ 343.0, 554.0, 332.0 }, Vec3{ -130.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, -105.0 }, light_mat.mat());
    try world.add(light.hittable());

    const center = Quad.init(Point3{ 0.0, 0.0, 0.0 }, Vec3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, white.mat());
    try world.add(center.hittable());

    const top = Quad.init(Point3{ 555.0, 555.0, 555.0 }, Vec3{ -555.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, -555.0 }, white.mat());
    try world.add(top.hittable());

    const bottom = Quad.init(Point3{ 0.0, 0.0, 555.0 }, Vec3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, white.mat());
    try world.add(bottom.hittable());

    // boxes
    var box1: Hittable = undefined;
    box1 = Box.init(Point3{ 0.0, 0.0, 0.0 }, Point3{ 165.0, 330.0, 165.0 }, white.mat()).hittable();
    box1 = RotateY.init(box1, 15.0).hittable();
    box1 = Translate.init(box1, Vec3{ 265.0, 0.0, 295.0 }).hittable();
    try world.add(box1);

    var box2: Hittable = undefined;
    box2 = Box.init(Point3{ 0.0, 0.0, 0.0 }, Point3{ 165.0, 165.0, 165.0 }, white.mat()).hittable();
    box2 = RotateY.init(box2, -18.0).hittable();
    box2 = Translate.init(box2, Vec3{ 130.0, 0.0, 65.0 }).hittable();
    try world.add(box2);

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 1.0;
    cam.image_width = 600;
    cam.samples_per_pixel = 200;
    cam.max_depth = 50;
    cam.vfov = 40;
    cam.look_from = Point3{ 278.0, 278.0, -800.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(bvh.hittable());
}

fn cornell_box_smoke() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    // materials
    const red = try Lambertian.init(allocator, Color{ 0.65, 0.05, 0.05 });
    const white = try Lambertian.init(allocator, Color{ 0.73, 0.73, 0.73 });
    const green = try Lambertian.init(allocator, Color{ 0.12, 0.45, 0.15 });
    const light_mat = try DiffuseLight.init_color(allocator, Color{ 7.0, 7.0, 7.0 });

    // walls
    const left = Quad.init(Point3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, green.mat());
    try world.add(left.hittable());

    const right = Quad.init(Point3{ 0.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, red.mat());
    try world.add(right.hittable());

    const light = Quad.init(Point3{ 113.0, 554.0, 127.0 }, Vec3{ 330.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, 305.0 }, light_mat.mat());
    try world.add(light.hittable());

    const center = Quad.init(Point3{ 0.0, 0.0, 0.0 }, Vec3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, 555.0 }, white.mat());
    try world.add(center.hittable());

    const top = Quad.init(Point3{ 555.0, 555.0, 555.0 }, Vec3{ -555.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, -555.0 }, white.mat());
    try world.add(top.hittable());

    const bottom = Quad.init(Point3{ 0.0, 0.0, 555.0 }, Vec3{ 555.0, 0.0, 0.0 }, Vec3{ 0.0, 555.0, 0.0 }, white.mat());
    try world.add(bottom.hittable());

    // boxes
    var box1: Hittable = undefined;
    box1 = Box.init(Point3{ 0.0, 0.0, 0.0 }, Point3{ 165.0, 330.0, 165.0 }, white.mat()).hittable();
    box1 = RotateY.init(box1, 15.0).hittable();
    box1 = Translate.init(box1, Vec3{ 265.0, 0.0, 295.0 }).hittable();
    const smoke1 = try ConstantMedium.init_color(allocator, box1, 0.01, Color{ 0.0, 0.0, 0.0 });
    try world.add(smoke1.hittable());

    var box2: Hittable = undefined;
    box2 = Box.init(Point3{ 0.0, 0.0, 0.0 }, Point3{ 165.0, 165.0, 165.0 }, white.mat()).hittable();
    box2 = RotateY.init(box2, -18.0).hittable();
    box2 = Translate.init(box2, Vec3{ 130.0, 0.0, 65.0 }).hittable();
    const smoke2 = try ConstantMedium.init_color(allocator, box2, 0.01, Color{ 1.0, 1.0, 1.0 });
    try world.add(smoke2.hittable());

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 1.0;
    cam.image_width = 600;
    cam.samples_per_pixel = 200;
    cam.max_depth = 50;
    cam.vfov = 40;
    cam.look_from = Point3{ 278.0, 278.0, -800.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(bvh.hittable());
}

fn all_features() !void {
    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    // world
    var world = HittableList.init(allocator);
    defer world.clear();

    // ground
    var ground_boxes = HittableList.init(allocator);
    const ground_mat = try Lambertian.init(allocator, Color{ 0.48, 0.83, 0.53 });
    const boxes_per_side = 20;
    for (0..boxes_per_side) |ui| {
        for (0..boxes_per_side) |uj| {
            const i = @as(f64, @floatFromInt(ui));
            const j = @as(f64, @floatFromInt(uj));

            const w: f64 = 100.0;
            const x0: f64 = -1000.0 + i * w;
            const z0: f64 = -1000.0 + j * w;
            const y0: f64 = 0.0;
            const x1: f64 = x0 + w;
            const y1: f64 = common.rand_f64(1.0, 101.0);
            const z1: f64 = z0 + w;

            const box = try allocator.create(Box);
            box.* = Box.init(Point3{ x0, y0, z0 }, Point3{ x1, y1, z1 }, ground_mat.mat());
            try ground_boxes.add(box.hittable());
        }
    }
    const ground_bvh = try BVHNode.init(allocator, ground_boxes);
    try world.add(ground_bvh.hittable());

    // light
    const light_mat = try DiffuseLight.init_color(allocator, Color{ 7.0, 7.0, 7.0 });
    const light = Quad.init(Point3{ 123.0, 554.0, 147.0 }, Vec3{ 300.0, 0.0, 0.0 }, Vec3{ 0.0, 0.0, 265.0 }, light_mat.mat());
    try world.add(light.hittable());

    // moving sphere
    const center1 = Point3{ 400.0, 400.0, 200.0 };
    const center2 = center1 + Point3{ 30.0, 0.0, 0.0 };
    const moving_sphere_mat = try Lambertian.init(allocator, Color{ 0.7, 0.3, 0.1 });
    const moving_sphere = Sphere.init_moving(center1, center2, 50.0, moving_sphere_mat.mat());
    try world.add(moving_sphere.hittable());

    // glass sphere
    const glass_mat = Dielectric{ .refractionIndex = 1.5 };
    const glass_sphere = Sphere.init(Point3{ 260.0, 150.0, 45.0 }, 50.0, glass_mat.mat());
    try world.add(glass_sphere.hittable());

    // metal sphere
    const metal = Metal.init(Color{ 0.8, 0.8, 0.9 }, 1.0);
    const sphere_metal = Sphere.init(Point3{ 0.0, 150.0, 145.0 }, 50.0, metal.mat());
    try world.add(sphere_metal.hittable());

    // really nice blue sphere
    const boundary = Sphere.init(Point3{ 360.0, 150.0, 145.0 }, 70.0, glass_mat.mat());
    try world.add(boundary.hittable());
    const x = try ConstantMedium.init_color(allocator, boundary.hittable(), 0.2, Color{ 0.2, 0.4, 0.9 });
    try world.add(x.hittable());

    // fog
    const fog_mat = Dielectric{ .refractionIndex = 1.5 };
    const boundary_fog = Sphere.init(Point3{ 0.0, 0.0, 0.0 }, 5000.0, fog_mat.mat());
    const fog = try ConstantMedium.init_color(allocator, boundary_fog.hittable(), 0.0001, Color{ 1.0, 1.0, 1.0 });
    try world.add(fog.hittable());

    // texture from image
    const earth_img = try allocator.create(Image);
    earth_img.* = try Image.init(allocator, "assets/textures/earth.jpg");
    const earth_mat = Lambertian.init_texture(earth_img.texture());
    const globe = Sphere.init(Point3{ 400.0, 200.0, 400.0 }, 100.0, earth_mat.mat());
    try world.add(globe.hittable());

    // noise
    const noise_tex = Noise.init(0.2);
    const noise_mat = Lambertian.init_texture(noise_tex.texture());
    const sphere_noise = Sphere.init(Point3{ 220.0, 280.0, 300.0 }, 80.0, noise_mat.mat());
    try world.add(sphere_noise.hittable());

    // translation and rotation
    const n = 1000;
    var little_spheres = HittableList.init(allocator);
    const white = try Lambertian.init(allocator, Color{ 0.73, 0.73, 0.73 });
    for (0..n) |_| {
        const sphere = try allocator.create(Sphere);
        sphere.* = Sphere.init(vec3.rand(0.0, 165.0), 10.0, white.mat());
        try little_spheres.add(sphere.hittable());
    }

    const bvh_little_spheres = try BVHNode.init(allocator, little_spheres);
    var hittable_little_sphere: Hittable = undefined;
    hittable_little_sphere = RotateY.init(bvh_little_spheres.hittable(), 15.0).hittable();
    hittable_little_sphere = Translate.init(hittable_little_sphere, Vec3{ -100.0, 270.0, 395.0 }).hittable();
    try world.add(hittable_little_sphere);

    const bvh = try BVHNode.init(allocator, world);

    // camera
    var cam = Camera{};
    cam.aspect_radio = 1.0;
    cam.image_width = 400;
    cam.samples_per_pixel = 250;
    cam.max_depth = 4;
    cam.vfov = 40;
    cam.look_from = Point3{ 478.0, 278.0, -600.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = 0.0;
    cam.focus_dist = 10.0;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(bvh.hittable());
}
