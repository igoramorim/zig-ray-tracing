const std = @import("std");

const Vec3 = @import("vec3.zig").Vec3;
const Interval = @import("interval.zig").Interval;

pub const Color = Vec3;

pub fn to_rgb(pixel_color: Color) RGB {
    // values in range 0.0 to 1.0
    var r: f64 = pixel_color.x;
    var g: f64 = pixel_color.y;
    var b: f64 = pixel_color.z;

    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    // values in range 0 to 255
    const intensity = Interval{ .min = 0.000, .max = 0.999 };
    const rbyte: u8 = @as(u8, @intFromFloat(256 * intensity.clamp(r)));
    const gbyte: u8 = @as(u8, @intFromFloat(256 * intensity.clamp(g)));
    const bbyte: u8 = @as(u8, @intFromFloat(256 * intensity.clamp(b)));

    return RGB{
        .r = rbyte,
        .g = gbyte,
        .b = bbyte,
    };
}

pub fn linear_to_gamma(linear_component: f64) f64 {
    if (linear_component > 0) {
        return std.math.sqrt(linear_component);
    }
    return 0;
}

pub const RGB = struct {
    r: u8,
    g: u8,
    b: u8,
};

pub const PPM = struct {
    width: usize,
    height: usize,
    data: []RGB,
    allocator: std.mem.Allocator,

    pub fn init(allocator: std.mem.Allocator, width: usize, height: usize) !PPM {
        return PPM{
            .width = width,
            .height = height,
            .data = try allocator.alloc(RGB, width * height),
            .allocator = allocator,
        };
    }

    pub fn deinit(self: *PPM) void {
        self.allocator.free(self.data);
    }

    pub fn write_all(self: PPM) !void {
        const stdout = std.io.getStdOut().writer();

        var bufwriter = std.io.bufferedWriter(stdout);
        var bwriter = bufwriter.writer();

        try bwriter.print("P3\n{d} {d}\n255\n", .{ self.width, self.height });
        for (self.data) |pixel| {
            try bwriter.print("{} {} {}\n", .{ pixel.r, pixel.g, pixel.b });
        }

        try bufwriter.flush();
    }
};
