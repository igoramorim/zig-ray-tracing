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

// TODO:
// - (zig) Remove the hittable() material() methods. Add an attr that holds it and it is initialized in init()
// - (zig) Parallelize
// - Add other types of lights (directional and spot)
// - Cast shadows

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
    const box1_sides = Box(Point3{ 130.0, 0.0, 65.0 }, Point3{ 295.0, 165.0, 230.0 }, white.mat());
    for (&box1_sides) |*side| {
        try world.add(side.hittable());
    }

    const box2_sides = Box(Point3{ 265.0, 0.0, 295.0 }, Point3{ 430.0, 330.0, 460.0 }, white.mat());
    for (&box2_sides) |*side| {
        try world.add(side.hittable());
    }

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
