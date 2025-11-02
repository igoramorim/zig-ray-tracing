const std = @import("std");
const stdout = std.io.getStdOut().writer();
const debug = std.debug;

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Vec3 = vec3.Vec3;
const f3 = vec3.f3;
const common = @import("common.zig");
const Ray = @import("ray.zig").Ray;
const HittableList = @import("hittable_list.zig").HittableList;
const color = @import("color.zig");
const Color = color.Color;
const PPM = color.PPM;
const HitRecord = @import("hittable.zig").HitRecord;
const Interval = @import("interval.zig").Interval;

pub const Camera = struct {
    aspect_radio: f64 = 1.0, // ratio of image width over height
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

    pub fn render(self: *Camera, world: HittableList) !void {
        self.initialize();

        var bufwriter = std.io.bufferedWriter(stdout);
        var bwriter = bufwriter.writer();
        try bwriter.print("P3\n{d} {d}\n255\n", .{ self.image_width, self.image_height });

        var j: u32 = 0;
        while (j < self.image_height) : (j = j + 1) {
            debug.print("scan lines remaining: {d}\n", .{(self.image_height - j)});

            var i: u32 = 0;
            while (i < self.image_width) : (i = i + 1) {
                var pixel_color = Color{ 0.0, 0.0, 0.0 };
                var sample: u32 = 0;

                // antialiasing: use multiple samples around the target pixel
                while (sample < self.samples_per_pixel) : (sample = sample + 1) {
                    const ray = self.get_ray(i, j);
                    pixel_color += self.ray_color(ray, self.max_depth, world);
                }

                pixel_color *= f3(self.pixel_samples_scale);
                try color.write(bwriter, pixel_color);
            }
        }

        try bufwriter.flush();
        debug.print("done!\n", .{});
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

    fn ray_color(self: Camera, ray: Ray, depth: u32, world: HittableList) Color {
        // if we've exceeded the ray bounce limit, no more light is gathered
        if (depth <= 0) {
            return Color{ 0.0, 0.0, 0.0 };
        }

        var rec = HitRecord{};
        if (world.hit(ray, Interval{ .min = 0.001, .max = common.infinity }, &rec)) {
            var scattered: Ray = undefined;
            var attenuation = Color{ 0.0, 0.0, 0.0 };

            if (rec.mat.scatter(ray, rec, &attenuation, &scattered)) {
                return attenuation * self.ray_color(scattered, depth - 1, world);
            }

            return Color{ 0.0, 0.0, 0.0 };
        }

        // if ray does not hit anything, render a background gradiant linearly from blue to white
        const unit_direction = vec3.unit_vector(ray.direction);
        const a: f64 = 0.5 * (unit_direction[1] + 1.0);

        const white = Color{ 1.0, 1.0, 1.0 };
        const blue = Color{ 0.5, 0.7, 1.0 };

        return f3(1.0 - a) * white + f3(a) * blue;
    }
};
