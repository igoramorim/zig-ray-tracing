const std = @import("std");

const Color = @import("color.zig").Color;
const Point3 = @import("vec3.zig").Point3;

pub const Texture = struct {
    ptr: *const anyopaque,
    value_fn: *const fn (ptr: *const anyopaque, u: f64, v: f64, p: Point3) Color,

    pub fn value(self: Texture, u: f64, v: f64, p: Point3) Color {
        return self.value_fn(self.ptr, u, v, p);
    }
};

pub const SolidColor = struct {
    const Self = @This();

    albedo: Color,

    pub fn init(albedo: Color) SolidColor {
        return SolidColor{ .albedo = albedo };
    }

    pub fn texture(self: *const SolidColor) Texture {
        return Texture{
            .ptr = self,
            .value_fn = value,
        };
    }

    pub fn value(ptr: *const anyopaque, _: f64, _: f64, _: Point3) Color {
        const self: *const Self = @ptrCast(@alignCast(ptr));
        return self.albedo;
    }
};

pub const Checker = struct {
    const Self = @This();

    inv_scale: f64,
    even: Texture,
    odd: Texture,

    pub fn init_texture(scale: f64, even: Texture, odd: Texture) Checker {
        return Checker{
            .inv_scale = @as(f64, 1.0) / scale,
            .even = even,
            .odd = odd,
        };
    }

    pub fn init_color(allocator: std.mem.Allocator, scale: f64, c1: Color, c2: Color) !Checker {
        const even = try allocator.create(SolidColor);
        even.* = SolidColor.init(c1);

        const odd = try allocator.create(SolidColor);
        odd.* = SolidColor.init(c2);

        return Checker{
            .inv_scale = @as(f64, 1.0) / scale,
            .even = even.texture(),
            .odd = odd.texture(),
        };
    }

    pub fn texture(self: *const Checker) Texture {
        return Texture{
            .ptr = self,
            .value_fn = value,
        };
    }

    pub fn value(ptr: *const anyopaque, u: f64, v: f64, p: Point3) Color {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        const x_int: i64 = @intFromFloat(@floor(self.inv_scale * p[0]));
        const y_int: i64 = @intFromFloat(@floor(self.inv_scale * p[1]));
        const z_int: i64 = @intFromFloat(@floor(self.inv_scale * p[2]));

        const is_even = @mod(x_int + y_int + z_int, 2) == 0;

        return if (is_even) self.even.value(u, v, p) else self.odd.value(u, v, p);
    }
};
