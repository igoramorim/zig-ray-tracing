const Point3 = @import("vec3.zig").Point3;
const common = @import("common.zig");

const point_count: i32 = 256;

pub const Perlin = struct {
    rand_float: [point_count]f64 = undefined,
    perm_x: [point_count]i32 = undefined,
    perm_y: [point_count]i32 = undefined,
    perm_z: [point_count]i32 = undefined,

    pub fn init() Perlin {
        var perlin = Perlin{};

        for (0..point_count) |i| {
            perlin.rand_float[i] = common.rand_f64_01();
        }

        generate_perm(&perlin.perm_x);
        generate_perm(&perlin.perm_y);
        generate_perm(&perlin.perm_z);

        return perlin;
    }

    fn generate_perm(p: *[point_count]i32) void {
        for (0..point_count) |i| {
            p[i] = @intCast(i);
        }

        permute(p, point_count);
    }

    fn permute(p: *[point_count]i32, n: i32) void {
        var i: usize = @intCast(n - 1);
        while (i > 0) : (i = i - 1) {
            const target: usize = @intCast(common.rand_i64(0, @intCast(i)));
            const tmp = p[i];
            p[i] = p[target];
            p[target] = tmp;
        }
    }

    pub fn noise(self: Perlin, p: Point3) f64 {
        const i: usize = @intCast((4 * @as(i32, @intFromFloat(p[0]))) & 255);
        const j: usize = @intCast((4 * @as(i32, @intFromFloat(p[1]))) & 255);
        const k: usize = @intCast((4 * @as(i32, @intFromFloat(p[2]))) & 255);

        const idx: usize = @intCast(self.perm_x[i] ^ self.perm_y[j] ^ self.perm_z[k]);
        return self.rand_float[idx];
    }
};
