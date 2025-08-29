const std = @import("std");
const stdout = std.io.getStdOut().writer();

const Vec3 = @import("vec3.zig").Vec3;
const Interval = @import("interval.zig").Interval;

pub const Color = Vec3;

pub fn write_color(pixel_color: Color) !void {
    // values in range 0.0 to 1.0
    var r: f64 = pixel_color.x;
    var g: f64 = pixel_color.y;
    var b: f64 = pixel_color.z;

    r = linear_to_gamma(r);
    g = linear_to_gamma(g);
    b = linear_to_gamma(b);

    // values in range 0 to 255
    const intensity = Interval{ .min = 0.000, .max = 0.999 };
    const rbyte: i64 = @as(i64, @intFromFloat(256 * intensity.clamp(r)));
    const gbyte: i64 = @as(i64, @intFromFloat(256 * intensity.clamp(g)));
    const bbyte: i64 = @as(i64, @intFromFloat(256 * intensity.clamp(b)));

    try stdout.print("{d} {d} {d}\n", .{ rbyte, gbyte, bbyte });
}

pub fn linear_to_gamma(linear_component: f64) f64 {
    if (linear_component > 0) {
        return std.math.sqrt(linear_component);
    }
    return 0;
}
