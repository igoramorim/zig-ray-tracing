const std = @import("std");
const stdout = std.io.getStdOut().writer();
const debug = std.debug;

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Vec3 = vec3.Vec3;
const f3 = vec3.f3;
const common = @import("common.zig");
const Ray = @import("ray.zig").Ray;
const Hittable = @import("hittable.zig").Hittable;
const color = @import("color.zig");
const Color = color.Color;
const HitRecord = @import("hittable.zig").HitRecord;
const Interval = @import("interval.zig").Interval;
const State = @import("main.zig").State;

pub const Camera = struct {
    aspect_radio: f64 = 1.0, // ratio of image eidth over height
    image_width: u32 = 100, // rendered image width in pixel count
    image_height: u32 = undefined, // rendered image height
    center: Point3 = Point3{ 0.0, 0.0, 0.0 }, // camera center - all rays will originate from here ("eye point")
    pixel_00_loc: Point3 = Point3{ 0.0, 0.0, 0.0 }, // location of pixel 0,0
    pixel_delta_u: Vec3 = Vec3{ 0.0, 0.0, 0.0 }, // offset to pixel to the right
    pixel_delta_v: Vec3 = Vec3{ 0.0, 0.0, 0.0 }, // off to pixel below
    samples_per_pixel: u32 = 10, // count of random samples for each pixel
    pixel_samples_scale: f64 = undefined, // color scale factor for a sum of pixel samples
    max_depth: u32 = 10, // maximum number of rays to bounce into scene
    vfov: f64 = 90, // vertical view angle (field of view)
    look_from: Point3 = Point3{ 0.0, 0.0, 0.0 }, // point the camera is looking from
    look_at: Point3 = Point3{ 0.0, 0.0, -1.0 }, // point the camera is looking at
    vup: Vec3 = Vec3{ 0.0, 1.0, 0.0 }, // camera-relative "up" direction
    u: Vec3 = undefined, // camera frame basis vectors
    v: Vec3 = undefined,
    w: Vec3 = undefined,
    defocus_angle: f64 = 0, // variation angle of rays through each pixel
    focus_dist: f64 = 10, // distance from camera lookfrom point to plane of perfect focus
    defocus_disk_u: Vec3 = undefined, // defocus disk horizontal radius
    defocus_disk_v: Vec3 = undefined, // defocus disk vertical radius
    background_color: Color = Color{ 0.70, 0.80, 1.0 },

    pub fn render(self: *Camera, allocator: std.mem.Allocator, world: Hittable, state: *State) !void {
        self.initialize();

        // try render_by_line(allocator, self, world, state);
        try render_by_tile(allocator, self, world, state);
    }

    fn initialize(self: *Camera) void {
        // calculate image height from image width and aspect radio
        const heightf: f64 = @as(f64, @floatFromInt(self.image_width)) / self.aspect_radio;
        self.image_height = @as(u32, @intFromFloat(heightf));
        if (self.image_height < 1) {
            self.image_height = 1;
        }

        self.pixel_samples_scale = 1.0 / @as(f64, @floatFromInt(self.samples_per_pixel));

        self.center = self.look_from;

        // determine viewport dimensions
        const theta = common.deg_to_rad(self.vfov);
        const h = @tan(theta / 2);
        const viewport_height: f64 = 2.0 * h * self.focus_dist;
        const viewport_width: f64 = viewport_height * (@as(f64, @floatFromInt(self.image_width)) / @as(f64, @floatFromInt(self.image_height)));

        // calculate the u,v,w unit basis vectors for the camera coordinate frame
        self.w = vec3.unit_vector(self.look_from - self.look_at);
        self.u = vec3.unit_vector(vec3.cross(self.vup, self.w));
        self.v = vec3.cross(self.w, self.u);

        // calculate the vectors across the horizontal and down the vertical viewport edges
        const viewport_u: Vec3 = self.u * f3(viewport_width); // vector across viewport horizontal edge
        const viewport_v: Vec3 = -self.v * f3(viewport_height); // vector down viewport vertical edge

        // calculate the horizontal and vertical delta vectors from pixel to pixel
        self.pixel_delta_u = viewport_u / f3(@as(f64, @floatFromInt(self.image_width)));
        self.pixel_delta_v = viewport_v / f3(@as(f64, @floatFromInt(self.image_height)));

        // calculate the location of the upper left pixel
        const viewport_upper_left: Vec3 = self.center - (self.w * f3(self.focus_dist)) - viewport_u / f3(2.0) - viewport_v / f3(2.0);
        self.pixel_00_loc = viewport_upper_left + f3(0.5) * (self.pixel_delta_u + self.pixel_delta_v);

        // calculate the camera defocus disk basis vectors
        const defocus_radius = self.focus_dist * @tan(common.deg_to_rad(self.defocus_angle / 2.0));
        self.defocus_disk_u = self.u * f3(defocus_radius);
        self.defocus_disk_v = self.v * f3(defocus_radius);
    }

    // construct a camera ray originating from the defocus disk and directed at randomdly sampled
    // point around the pixel location i, j
    fn get_ray(self: Camera, i: u32, j: u32) Ray {
        const offset = sample_square();

        const ixu: Vec3 = f3(@as(f64, @floatFromInt(i)) + offset[0]) * self.pixel_delta_u;
        const jyv: Vec3 = f3(@as(f64, @floatFromInt(j)) + offset[1]) * self.pixel_delta_v;
        const pixel_sample: Vec3 = self.pixel_00_loc + ixu + jyv;

        const ray_origin = if (self.defocus_angle <= 0) self.center else self.defocus_disk_sample();
        const ray_direction = pixel_sample - ray_origin;
        const ray_time = common.rand_f64_01();

        return Ray{ .origin = ray_origin, .direction = ray_direction, .time = ray_time };
    }

    // returns the vector to a random point in the [-.5, -.5]-[+.5, +.5] unit square
    fn sample_square() Vec3 {
        return Vec3{
            common.rand_f64_01() - 0.5,
            common.rand_f64_01() - 0.5,
            0.0,
        };
    }

    // returns a random point in the camera defocus disk
    fn defocus_disk_sample(self: Camera) Point3 {
        const p = vec3.rand_in_unit_disk();
        return self.center + (f3(p[0]) * self.defocus_disk_u) + (f3(p[1]) * self.defocus_disk_v);
    }

    fn ray_color(self: Camera, ray: Ray, depth: u32, world: Hittable) Color {
        // if we've exceeded the ray bounce limit, no more light is gathered
        if (depth <= 0) {
            return Color{ 0.0, 0.0, 0.0 };
        }

        var rec = HitRecord{};
        // if the ray hits nothing, returns the background color
        if (!world.hit(ray, Interval{ .min = 0.001, .max = common.infinity }, &rec)) {
            return self.background_color;
        }

        var scattered: Ray = undefined;
        var attenuation = Color{ 0.0, 0.0, 0.0 };
        const color_from_emission = rec.mat.emitted(rec.u, rec.v, rec.p);

        if (!rec.mat.scatter(ray, rec, &attenuation, &scattered)) {
            return color_from_emission;
        }

        const color_from_scatter = attenuation * self.ray_color(scattered, depth - 1, world);

        return color_from_emission + color_from_scatter;
    }
};

fn render_by_line(allocator: std.mem.Allocator, camera: *Camera, world: Hittable, state: *State) !void {
    var pool: std.Thread.Pool = undefined;
    try pool.init(.{ .allocator = allocator });
    defer pool.deinit();

    var wg = std.Thread.WaitGroup{};
    var lines_remaining = std.atomic.Value(u32).init(camera.image_height);

    var j: u32 = 0;
    while (j < camera.image_height) : (j = j + 1) {
        pool.spawnWg(&wg, render_line, .{RenderLineTask{
            .camera = camera,
            .world = world,
            .line_y = @as(u32, @intCast(j)),
            .state = state,
            .lines_remaining = &lines_remaining,
        }});
    }

    pool.waitAndWork(&wg);
}

const RenderLineTask = struct {
    camera: *Camera,
    world: Hittable,
    line_y: u32,
    state: *State,
    lines_remaining: *std.atomic.Value(u32),
};

fn render_line(task: RenderLineTask) void {
    // TODO: better moment to check the cancel?
    // if the current line is heavy, the user experience a delay between the
    // cancel and the start of the new render
    if (task.state.cancel_render.load(.monotonic)) {
        return;
    }

    var i: u32 = 0;
    while (i < task.camera.image_width) : (i = i + 1) {
        var pixel_color = Color{ 0.0, 0.0, 0.0 };
        var sample: u32 = 0;

        // antialiasing: use multiple samples around the target pixel
        while (sample < task.camera.samples_per_pixel) : (sample = sample + 1) {
            const ray = task.camera.get_ray(i, task.line_y);
            pixel_color += task.camera.ray_color(ray, task.camera.max_depth, task.world);
        }

        const idx = (task.line_y * task.camera.image_width + i) * 3;
        pixel_color *= f3(task.camera.pixel_samples_scale);
        task.state.buffer[idx + 0] = color.to_byte(pixel_color[0]);
        task.state.buffer[idx + 1] = color.to_byte(pixel_color[1]);
        task.state.buffer[idx + 2] = color.to_byte(pixel_color[2]);
    }

    const lines_remaining = task.lines_remaining.fetchSub(1, .monotonic) - 1;
    const lines_done = task.camera.image_height - lines_remaining;
    task.state.progress = @intCast(((lines_done * 100) / task.camera.image_height));
}

fn render_by_tile(allocator: std.mem.Allocator, camera: *Camera, world: Hittable, state: *State) !void {
    var pool: std.Thread.Pool = undefined;
    try pool.init(.{ .allocator = allocator });
    defer pool.deinit();

    var wg = std.Thread.WaitGroup{};

    const tile_size: u8 = 16;
    const tiles_x: u32 = @intCast(camera.image_width / tile_size);
    const tiles_y: u32 = @intCast(camera.image_height / tile_size);
    const total_tiles: u32 = tiles_x * tiles_y;
    var tiles_remaining = std.atomic.Value(u32).init(total_tiles);

    var j: u32 = 0;
    while (j < tiles_y) : (j = j + 1) {
        var i: u32 = 0;
        while (i < tiles_x) : (i = i + 1) {
            pool.spawnWg(&wg, render_tile, .{RenderTileTask{
                .camera = camera,
                .world = world,
                .x_start = i * tile_size,
                .x_end = (i * tile_size) + tile_size,
                .y_start = j * tile_size,
                .y_end = (j * tile_size) + tile_size,
                .state = state,
                .total_tiles = total_tiles,
                .tiles_remaining = &tiles_remaining,
            }});
        }
    }

    pool.waitAndWork(&wg);
}

const RenderTileTask = struct {
    camera: *Camera,
    world: Hittable,
    x_start: u32,
    x_end: u32,
    y_start: u32,
    y_end: u32,
    state: *State,
    total_tiles: u32,
    tiles_remaining: *std.atomic.Value(u32),
};

fn render_tile(task: RenderTileTask) void {
    if (task.state.cancel_render.load(.monotonic)) {
        return;
    }

    var j: u32 = task.y_start;
    while (j < task.y_end) : (j = j + 1) {
        var i: u32 = task.x_start;
        while (i < task.x_end) : (i = i + 1) {
            var pixel_color = Color{ 0.0, 0.0, 0.0 };
            var sample: u32 = 0;

            // antialiasing: use multiple samples around the target pixel
            while (sample < task.camera.samples_per_pixel) : (sample = sample + 1) {
                const ray = task.camera.get_ray(i, j);
                pixel_color += task.camera.ray_color(ray, task.camera.max_depth, task.world);
            }

            const idx = (j * task.camera.image_width + i) * 3;
            pixel_color *= f3(task.camera.pixel_samples_scale);
            task.state.buffer[idx + 0] = color.to_byte(pixel_color[0]);
            task.state.buffer[idx + 1] = color.to_byte(pixel_color[1]);
            task.state.buffer[idx + 2] = color.to_byte(pixel_color[2]);
        }
    }

    const tiles_remaining = task.tiles_remaining.fetchSub(1, .monotonic) - 1;
    const tiles_done = task.total_tiles - tiles_remaining;
    task.state.progress = @intCast(((tiles_done * 100) / task.total_tiles));
}
