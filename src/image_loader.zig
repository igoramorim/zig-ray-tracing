const std = @import("std");

const stbi = @import("c.zig").stbi;

const ImageError = error{
    ImageLoadFailed,
};

pub const ImageLoader = struct {
    width: i32 = 0,
    height: i32 = 0,
    bytes_per_pixel: u8 = 3,
    bytes_per_scanline: i32 = 0,
    fdata: ?[*]f32 = null, // Linear floating point pixel data
    bdata: []u8 = undefined, // Linear 8-bit pixel data

    pub fn load(self: *ImageLoader, allocator: std.mem.Allocator, filename: [:0]const u8) !void {
        var n: c_int = self.bytes_per_pixel;

        self.fdata = stbi.stbi_loadf(filename.ptr, &self.width, &self.height, &n, self.bytes_per_pixel);
        if (self.fdata == null) return ImageError.ImageLoadFailed;

        self.bytes_per_scanline = self.width * self.bytes_per_pixel;
        try self.convert_to_bytes(allocator);
    }

    fn convert_to_bytes(self: *ImageLoader, allocator: std.mem.Allocator) !void {
        if (self.fdata) |ptr| {
            const total_bytes = @as(usize, @intCast(self.width * self.height * self.bytes_per_pixel));
            self.bdata = try allocator.alloc(u8, total_bytes);

            const float_slice = ptr[0..total_bytes];
            for (float_slice, 0..) |fval, i| {
                self.bdata[i] = float_to_byte(fval);
            }
        } else unreachable;
    }

    fn float_to_byte(f: f32) u8 {
        if (f <= 0.0) return 0;
        if (1.0 <= f) return 255;
        return @intFromFloat(f * 256.0);
    }

    pub fn deinit(self: ImageLoader) void {
        stbi.stbi_free(self.fdata);
    }

    pub fn pixel_data(self: ImageLoader, x: i64, y: i64) [3]u8 {
        const magenta = [_]u8{ 255, 0, 255 };
        if (self.bdata.len == 0) return magenta;

        const cx = @as(usize, @intCast(clamp(x, 0, self.width)));
        const x_offset = cx * @as(usize, @intCast(self.bytes_per_pixel));

        const cy = @as(usize, @intCast(clamp(y, 0, self.height)));
        const y_offset = cy * @as(usize, @intCast(self.bytes_per_scanline));

        const idx = y_offset + x_offset;
        return self.bdata[idx..][0..3].*;
    }

    /// Return the value clamped to the range [low, high)
    fn clamp(x: i64, low: i64, high: i64) i64 {
        if (x < low) return low;
        if (x < high) return x;
        return high - 1;
    }
};
