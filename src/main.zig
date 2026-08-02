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
const Noise = texture.Noise;
const Quad = @import("hittable.zig").Quad;
const DiffuseLight = @import("material.zig").DiffuseLight;
const Box = @import("hittable.zig").Box;
const RotateY = @import("hittable.zig").RotateY;
const Translate = @import("hittable.zig").Translate;
const ConstantMedium = @import("hittable.zig").ConstantMedium;

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zopengl = @import("zopengl");

const width = 800;
const height = 800;

const default_samples_per_pixel = 30;
const min_samples_per_pixel = 1;
const max_samples_per_pixel = 10_000;

const default_max_depth = 20;
const min_max_depth = 1;
const max_max_depth = 100;

const default_fov = 20.0;
const min_fov = 1.0;
const max_fov = 90.0;

const default_defocus_angle = 0.0;
const min_defocus_angle = 0.0;
const max_defocus_angle = 10.0;

const default_focus_dist = 10.0;
const min_focus_dist = 1.0;
const max_focus_dist = 100.0;

// TODO:
// - (zig) Remove the hittable() material() methods. Add an attr that holds it and it is initialized in init()
// - (zig) Parallelize
// - Add other types of lights (directional and spot)
// - Cast shadows
// - Scale transformation
// - Rotate in X and Z axis

pub fn main() !void {
    try zglfw.init();
    defer zglfw.terminate();

    const gl_major = 3;
    const gl_minor = 3;
    zglfw.windowHint(.context_version_major, gl_major);
    zglfw.windowHint(.context_version_minor, gl_minor);
    zglfw.windowHint(.opengl_profile, .opengl_core_profile);
    zglfw.windowHint(.opengl_forward_compat, true);
    zglfw.windowHint(.client_api, .opengl_api);
    zglfw.windowHint(.resizable, false);

    const window = try zglfw.Window.create(width, height, "zig ray tracer", null, null);
    defer window.destroy();

    zglfw.makeContextCurrent(window);
    zglfw.swapInterval(1);

    try zopengl.loadCoreProfile(zglfw.getProcAddress, gl_major, gl_minor);
    const gl = zopengl.bindings;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    zgui.init(allocator);
    defer zgui.deinit();

    zgui.backend.init(window);
    defer zgui.backend.deinit();

    const total_pixels: usize = width * height * 3; // RGB
    var state = try State.init(allocator, total_pixels);
    defer state.deinit();
    var render_thread: ?std.Thread = null;
    defer {
        // make sure to cancel the render if the app is closed while rendering
        if (render_thread) |thread| {
            state.cancel_render.store(true, .monotonic);
            thread.join();
        }
    }

    var tex_id: gl.Uint = 0;
    gl.genTextures(1, &tex_id);
    gl.bindTexture(gl.TEXTURE_2D, tex_id);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, width, height, 0, gl.RGB, gl.UNSIGNED_BYTE, null);
    gl.bindTexture(gl.TEXTURE_2D, 0);

    var framebuf_id: gl.Uint = 0;
    gl.genFramebuffers(1, &framebuf_id);
    gl.bindFramebuffer(gl.READ_FRAMEBUFFER, framebuf_id);
    gl.framebufferTexture2D(gl.READ_FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex_id, 0);
    gl.bindFramebuffer(gl.READ_FRAMEBUFFER, 0);

    var opts = Opts{
        .width = width,
    };
    var samples_per_pixel: i32 = default_samples_per_pixel;
    var max_depth: i32 = default_max_depth;
    var fov: f32 = default_fov;
    var defocus_angle: f32 = default_defocus_angle;
    var focus_dist: f32 = default_focus_dist;

    var current_selection: ?usize = null;
    const scenes_text = [_][:0]const u8{
        "bouncing spheres",
        "checkered spheres",
        "earth",
        "perlin spheres",
        "quads",
        "diffuse light",
        "empty cornell box",
        "cornell box",
        "cornell box smoke",
        "all features",
    };

    while (!window.shouldClose() and window.getKey(.escape) != .press) {
        zglfw.pollEvents();

        gl.bindTexture(gl.TEXTURE_2D, tex_id);
        gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, width, height, gl.RGB, gl.UNSIGNED_BYTE, state.buffer.ptr);

        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, framebuf_id);
        gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, 0);
        // destination height and width are swapped so OpenGL does not render upside down
        gl.blitFramebuffer(0, 0, width, height, 0, height, width, 0, gl.COLOR_BUFFER_BIT, gl.NEAREST);

        const fb_size = window.getFramebufferSize();
        zgui.backend.newFrame(@intCast(fb_size[0]), @intCast(fb_size[1]));

        if (zgui.begin("Settings", .{})) {
            const preview_text = if (current_selection != null)
                scenes_text[current_selection.?]
            else
                "Choose a scene";

            if (zgui.beginCombo("Scene", .{ .preview_value = preview_text })) {
                defer zgui.endCombo();

                for (scenes_text, 0..) |text, index| {
                    const is_selected = (current_selection == index);
                    if (zgui.selectable(text, .{ .selected = is_selected })) {
                        current_selection = index;
                    }
                }
            }

            if (zgui.sliderInt("Samples per pixel", .{
                .v = &samples_per_pixel,
                .min = min_samples_per_pixel,
                .max = max_samples_per_pixel,
            })) {
                opts.samples_per_pixel = @intCast(samples_per_pixel);
            }

            if (zgui.sliderInt("Max depth", .{
                .v = &max_depth,
                .min = min_max_depth,
                .max = max_max_depth,
            })) {
                opts.max_depth = @intCast(max_depth);
            }

            if (zgui.sliderFloat("FOV", .{
                .v = &fov,
                .min = min_fov,
                .max = max_fov,
            })) {
                opts.fov = @floatCast(fov);
            }

            if (zgui.sliderFloat("Defocus Angle", .{
                .v = &defocus_angle,
                .min = min_defocus_angle,
                .max = max_defocus_angle,
            })) {
                opts.defocus_angle = @floatCast(defocus_angle);
            }

            if (zgui.sliderFloat("Focus Distance", .{
                .v = &focus_dist,
                .min = min_focus_dist,
                .max = max_focus_dist,
            })) {
                opts.focus_dist = @floatCast(focus_dist);
            }

            if (zgui.button("Render", .{})) {
                if (current_selection) |selection| {
                    state.cancel_render.store(true, .monotonic); // cancels the current render

                    if (render_thread) |thread| { // wait the thread finishes
                        thread.join();
                        render_thread = null;
                    }

                    @memset(state.buffer, 0); // clear the buffer
                    state.cancel_render.store(false, .monotonic); // reset the cancel flag

                    // start a new render
                    state.lines_remaining = 0;
                    const scene = get_scene_fn(selection);
                    render_thread = try std.Thread.spawn(
                        .{},
                        scene_runner,
                        .{ allocator, scene, opts, &state },
                    );
                }
            }

            zgui.text("Lines remaining: {d}", .{state.lines_remaining});
        }
        zgui.end();
        zgui.backend.draw();

        window.swapBuffers();
    }
}

pub const State = struct {
    allocator: std.mem.Allocator,
    buffer: []u8,
    cancel_render: std.atomic.Value(bool),
    lines_remaining: u32 = 0,

    pub fn init(allocator: std.mem.Allocator, total_pixels: usize) !State {
        const buffer = try allocator.alloc(u8, total_pixels);
        @memset(buffer, 0); // clear the buffer

        return .{
            .allocator = allocator,
            .buffer = buffer,
            .cancel_render = std.atomic.Value(bool).init(false),
        };
    }

    pub fn deinit(self: *State) void {
        self.allocator.free(self.buffer);
    }
};

const scene_fn = *const fn (allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void;

fn scene_runner(allocator: std.mem.Allocator, scene: scene_fn, opts: Opts, state: *State) anyerror!void {
    try scene(allocator, opts, state);
}

fn get_scene_fn(id: usize) scene_fn {
    switch (id) {
        0 => return &bouncing_spheres,
        1 => return &checkered_spheres,
        2 => return &earth,
        3 => return &perlin_spheres,
        4 => return &quads,
        5 => return &diffuse_light,
        6 => return &empty_cornell_box,
        7 => return &cornell_box,
        8 => return &cornell_box_smoke,
        9 => return &all_features,
        else => unreachable,
    }
}

const Opts = struct {
    width: u32 = width,
    samples_per_pixel: u32 = default_samples_per_pixel,
    max_depth: u32 = default_max_depth,
    fov: f64 = default_fov,
    defocus_angle: f64 = default_defocus_angle,
    focus_dist: f64 = default_focus_dist,
};

fn bouncing_spheres(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 13.0, 2.0, 3.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;

    try cam.render(allocator, bvh.hittable(), state);
}

fn checkered_spheres(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 13.0, 2.0, 3.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;

    try cam.render(allocator, bvh.hittable(), state);
}

fn earth(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 0.0, 0.0, 12.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;

    try cam.render(allocator, bvh.hittable(), state);
}

fn perlin_spheres(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 13.0, 2.0, 3.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;

    try cam.render(allocator, bvh.hittable(), state);
}

fn quads(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 0.0, 0.0, 9.0 };
    cam.look_at = Point3{ 0.0, 0.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;

    try cam.render(allocator, bvh.hittable(), state);
}

fn diffuse_light(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 20.0, 3.0, 6.0 };
    cam.look_at = Point3{ 0.0, 2.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(allocator, bvh.hittable(), state);
}

fn empty_cornell_box(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 278.0, 278.0, -800.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(allocator, bvh.hittable(), state);
}

fn cornell_box(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 278.0, 278.0, -800.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(allocator, bvh.hittable(), state);
}

fn cornell_box_smoke(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 278.0, 278.0, -800.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(allocator, bvh.hittable(), state);
}

fn all_features(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
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
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = Point3{ 478.0, 278.0, -600.0 };
    cam.look_at = Point3{ 278.0, 278.0, 0.0 };
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;
    cam.background_color = Color{ 0.0, 0.0, 0.0 };

    try cam.render(allocator, bvh.hittable(), state);
}
