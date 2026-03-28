const std = @import("std");

const Color = @import("color.zig").Color;
const Point3 = @import("vec3.zig").Point3;
const ImageLoader = @import("image_loader.zig").ImageLoader;
const Interval = @import("interval.zig").Interval;

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

pub const Image = struct {
    const Self = @This();

    img: ImageLoader,

    pub fn init(allocator: std.mem.Allocator, filename: [:0]const u8) !Image {
        var loader = ImageLoader{};
        try loader.load(allocator, filename);
        return Image{ .img = loader };
    }

    pub fn texture(self: *const Image) Texture {
        return Texture{
            .ptr = self,
            .value_fn = value,
        };
    }

    pub fn value(ptr: *const anyopaque, u: f64, v: f64, _: Point3) Color {
        const self: *const Self = @ptrCast(@alignCast(ptr));

        // if there is no texture data, then return solid cyan as debugging tool
        if (self.img.height <= 0) return Color{ 0.0, 1.0, 0.0 };

        // clamp input texture coordinates to [0,1] x [1,0]
        const interval_u = Interval{ .min = 0, .max = 1 };
        const u_copy = interval_u.clamp(u);
        const interval_v = Interval{ .min = 0, .max = 1 };
        const v_copy = 1.0 - interval_v.clamp(v); // flip V to image coordinates

        const i = @as(i64, @intFromFloat(u_copy * @as(f64, @floatFromInt(self.img.width))));
        const j = @as(i64, @intFromFloat(v_copy * @as(f64, @floatFromInt(self.img.height))));
        const pixel = self.img.pixel_data(i, j);

        const color_scale_value: f64 = 1.0 / 255.0;
        const color_scale = Color{ color_scale_value, color_scale_value, color_scale_value };
        const color_pixel = Color{
            @as(f64, @floatFromInt(pixel[0])),
            @as(f64, @floatFromInt(pixel[1])),
            @as(f64, @floatFromInt(pixel[2])),
        };
        return color_scale * color_pixel;
    }
};
