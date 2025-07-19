const std = @import("std");
const debug = std.debug;

pub fn main() !void {
    const aspectRadio: f64 = 16.0 / 9.0;
    const imageWidth: i64 = 400;

    // calculate the image height and ensure that it is at least 1
    var imageHeight: i64 = imageWidth / aspectRadio;
    if (imageHeight < 1) {
        imageHeight = 1;
    }

    // camera
    const focalLength: f64 = 1.0;
    const viewportHeight: f64 = 2.0;
    const viewportWidth: f64 = viewportHeight * (@as(f64, @floatFromInt(imageWidth)) / @as(f64, @floatFromInt(imageHeight)));
    const cameraCenter = Point3{};
    debug.print("viewport width: {any}\n", .{viewportWidth});

    // calculate the vectors across the horizontal and down the vertical viewport edges
    const viewportU = Vec3{ .x = viewportWidth };
    const viewportV = Vec3{ .y = -viewportHeight };
    debug.print("viewport U: {any}\n", .{viewportU});
    debug.print("viewport V: {any}\n", .{viewportV});

    // calculate the horizontal and vertical delta vectors from pixel to pixel
    const pixelDeltaU = viewportU.DivI64(imageWidth);
    const pixelDeltaV = viewportV.DivI64(imageHeight);
    debug.print("pixel delta U: {any}\n", .{pixelDeltaU});
    debug.print("pixel delta V: {any}\n", .{pixelDeltaV});

    // calculate the location of the upper left pixel
    const viewportUpperLeft = cameraCenter.Sub(Vec3{ .z = focalLength }).Sub(viewportU.DivI64(2)).Sub(viewportV.DivI64(2));
    const pixel00Loc = pixelDeltaU.Add(pixelDeltaV).MultF64(0.5).Add(viewportUpperLeft);
    debug.print("viewport upper left: {any}\n", .{viewportUpperLeft});
    debug.print("pixel 00 loc: {any}\n", .{pixel00Loc});

    const stdout = std.io.getStdOut().writer();

    try stdout.print("P3\n{d} {d}\n255\n", .{ imageWidth, imageHeight });

    var j: i64 = 0;
    while (j < imageHeight) : (j = j + 1) {
        debug.print("scan lines remaining: {d}\n", .{(imageHeight - j)});

        var i: i64 = 0;
        while (i < imageWidth) : (i = i + 1) {
            const pixelCenter = pixel00Loc.Add(pixelDeltaU.MultI64(i)).Add(pixelDeltaV.MultI64(j));
            const rayDirection = pixelCenter.Sub(cameraCenter);
            const ray = Ray{ .orig = cameraCenter, .dir = rayDirection };
            // debug.print("pixel center: {any}\n", .{pixelCenter});
            // debug.print("camera center: {any}\nray direction: {any}\n", .{ cameraCenter, rayDirection });

            const pixelColor = rayColor(ray);
            // debug.print("pixel color: {any}\n", .{pixelColor});
            try writeColor(pixelColor);
            // debug.print("\n", .{});
        }
    }

    debug.print("done!\n", .{});
}

const Vec3 = struct {
    x: f64 = 0.0,
    y: f64 = 0.0,
    z: f64 = 0.0,

    pub fn X(self: Vec3) f64 {
        return self.x;
    }

    pub fn Y(self: Vec3) f64 {
        return self.y;
    }

    pub fn Z(self: Vec3) f64 {
        return self.z;
    }

    pub fn Length(self: Vec3) f64 {
        return std.math.sqrt(self.LengthSquared());
    }

    pub fn LengthSquared(self: Vec3) f64 {
        return (self.x * self.x) + (self.y * self.y) + (self.z * self.z);
    }

    pub fn DivI64(self: Vec3, i: i64) Vec3 {
        const value: f64 = 1 / @as(f64, @floatFromInt(i));
        return self.MultF64(value);
    }

    pub fn DivF64(self: Vec3, f: f64) Vec3 {
        const value: f64 = 1 / f;
        return self.MultF64(value);
    }

    pub fn Mult(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x * v.x,
            .y = self.y * v.y,
            .z = self.z * v.z,
        };
    }

    pub fn MultI64(self: Vec3, i: i64) Vec3 {
        return Vec3{
            .x = self.x * @as(f64, @floatFromInt(i)),
            .y = self.y * @as(f64, @floatFromInt(i)),
            .z = self.z * @as(f64, @floatFromInt(i)),
        };
    }

    pub fn MultF64(self: Vec3, f: f64) Vec3 {
        return Vec3{
            .x = f * self.x,
            .y = f * self.y,
            .z = f * self.z,
        };
    }

    pub fn Sub(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x - v.x,
            .y = self.y - v.y,
            .z = self.z - v.z,
        };
    }

    pub fn Add(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x + v.x,
            .y = self.y + v.y,
            .z = self.z + v.z,
        };
    }

    pub fn AddF64(self: Vec3, f: f64) Vec3 {
        return Vec3{
            .x = self.x + f,
            .y = self.y + f,
            .z = self.z + f,
        };
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

fn rayColor(r: Ray) Color {
    const t: f64 = hitSphere(Vec3{ .z = -1.0 }, 0.5, r);
    if (t > 0.0) {
        const normal: Vec3 = unitVector(r.At(t).Sub(Point3{ .z = -1.0 }));
        // remap range [-1 1] to [0 1] and map it to RGB
        const color = Color{
            .x = normal.X() + 1,
            .y = normal.Y() + 1,
            .z = normal.Z() + 1,
        };
        return color.MultF64(0.5);
    }

    const unitDirection = unitVector(r.Direction());
    const a: f64 = 0.5 * (unitDirection.Y() + 1.0);

    const white = Color{ .x = 1.0, .y = 1.0, .z = 1.0 };
    const blue = Color{ .x = 0.5, .y = 0.7, .z = 1.0 };

    return white.MultF64(1.0 - a).Add(blue.MultF64(a));
}

fn hitSphere(center: Vec3, radius: f64, ray: Ray) f64 {
    const oc: Vec3 = center.Sub(ray.Origin());
    const a: f64 = ray.Direction().LengthSquared();
    const h: f64 = dot(ray.Direction(), oc);
    const c: f64 = oc.LengthSquared() - (radius * radius);
    const discriminant: f64 = (h * h) - (a * c);

    if (discriminant < 0.0) {
        return -1.0;
    }

    return (h - std.math.sqrt(discriminant)) / a;
}

fn dot(u: Vec3, v: Vec3) f64 {
    return u.X() * v.X() +
        u.Y() * v.Y() +
        u.Z() * v.Z();
}

fn unitVector(v: Vec3) Vec3 {
    return v.DivF64(v.Length());
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
        return self.dir.MultF64(t).Add(self.orig);
    }
};
