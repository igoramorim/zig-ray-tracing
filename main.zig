const std = @import("std");

pub fn main() !void {
    const width = 256;
    const height = 256;

    const stdout = std.io.getStdOut().writer();
    const debug = std.debug;

    try stdout.print("P3\n{d} {d}\n255\n", .{ width, height });

    for (0..height) |j| {
        debug.print("scan lines remaining: {d}\n", .{(height - j)});

        for (0..width) |i| {
            const r: f64 = @as(f64, @floatFromInt(i)) / (width - 1);
            const g: f64 = @as(f64, @floatFromInt(j)) / (height - 1);
            const b: f64 = 0.0;

            // const pixelColor = Vec3{ .x = r, .y = g, .z = b };
            const pixelColor = Color{ .e = [3]f64{ r, g, b } };
            try writeColor(pixelColor);
        }
    }

    debug.print("done!\n", .{});
}

const Vec3 = struct {
    e: [3]f64 = [3]f64{ 0.0, 0.0, 0.0 },
    // x: f64 = 0.0,
    // y: f64 = 0.0,
    // z: f64 = 0.0,

    pub fn X(self: Vec3) f64 {
        return self.e[0];
        // return self.x;
    }

    pub fn Y(self: Vec3) f64 {
        return self.e[1];
        // return self.y;
    }

    pub fn Z(self: Vec3) f64 {
        return self.e[2];
        // return self.z;
    }
};

const Point3 = Vec3;

const Color = Vec3;

fn writeColor(pixelColor: Color) !void {
    // values in range 0.0 to 1.0
    const r: f64 = pixelColor.X();
    const g: f64 = pixelColor.Y();
    const b: f64 = pixelColor.Z();

    // values in range 0 to 255
    const rbyte: i64 = @as(i64, @intFromFloat(255.999 * r));
    const gbyte: i64 = @as(i64, @intFromFloat(255.999 * g));
    const bbyte: i64 = @as(i64, @intFromFloat(255.999 * b));

    const stdout = std.io.getStdOut().writer();
    try stdout.print("{d} {d} {d}\n", .{ rbyte, gbyte, bbyte });
}

const Ray = struct {
    orig: Point3,
    dir: Vec3,

    pub fn Origin(self: Ray) Point3 {
        return self.orig;
    }

    pub fn Direction(self: Ray) Vec3 {
        return self.dir;
    }

    pub fn At(self: Ray, t: f64) Point3 {
        return self.orig + (t * self.dir); // NOTE: self.dir.Mult(t) ?
    }
};
