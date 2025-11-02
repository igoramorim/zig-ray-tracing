const std = @import("std");

const Vec3 = @import("vec3.zig").Vec3;
const Interval = @import("interval.zig").Interval;

pub const Color = Vec3;

pub fn write(writer: anytype, pixel_color: Color) !void {
    // values in range 0.0 to 1.0
    var r: f64 = pixel_color[0];
    var g: f64 = pixel_color[1];
    var b: f64 = pixel_color[2];

    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    // values in range 0 to 255
    const intensity = Interval{ .min = 0.000, .max = 0.999 };
    const rbyte: u8 = @as(u8, @intFromFloat(256 * intensity.clamp(r)));
    const gbyte: u8 = @as(u8, @intFromFloat(256 * intensity.clamp(g)));
    const bbyte: u8 = @as(u8, @intFromFloat(256 * intensity.clamp(b)));

    try writer.print("{} {} {}\n", .{ rbyte, gbyte, bbyte });
}

pub fn linear_to_gamma(linear_component: f64) f64 {
    if (linear_component > 0) {
        return std.math.sqrt(linear_component);
    }
    return 0;
}
