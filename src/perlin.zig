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
        const u: f64 = p[0] - @floor(p[0]);
        const v: f64 = p[1] - @floor(p[1]);
        const w: f64 = p[2] - @floor(p[2]);

        const i = @as(i32, @intFromFloat(@floor(p[0])));
        const j = @as(i32, @intFromFloat(@floor(p[1])));
        const k = @as(i32, @intFromFloat(@floor(p[2])));

        var c: [2][2][2]f64 = undefined;

        for (0..2) |di| {
            for (0..2) |dj| {
                for (0..2) |dk| {
                    const idx_i: usize = @as(usize, @as(u32, @bitCast(i + @as(i32, @intCast(di))))) & 255;
                    const idx_j: usize = @as(usize, @as(u32, @bitCast(j + @as(i32, @intCast(dj))))) & 255;
                    const idx_k: usize = @as(usize, @as(u32, @bitCast(k + @as(i32, @intCast(dk))))) & 255;
                    const idx: usize = @intCast(self.perm_x[idx_i] ^ self.perm_y[idx_j] ^ self.perm_z[idx_k]);
                    c[di][dj][dk] = self.rand_float[idx];
                }
            }
        }

        return trilinear_interp(c, u, v, w);
    }

    fn trilinear_interp(c: [2][2][2]f64, u: f64, v: f64, w: f64) f64 {
        var accum: f64 = 0;
        for (0..2) |ui| {
            for (0..2) |uj| {
                for (0..2) |uk| {
                    const i: f64 = @as(f64, @floatFromInt(ui));
                    const j: f64 = @as(f64, @floatFromInt(uj));
                    const k: f64 = @as(f64, @floatFromInt(uk));

                    const val_i: f64 = i * u + (1.0 - i) * (1.0 - u);
                    const val_j: f64 = j * v + (1.0 - j) * (1.0 - v);
                    const val_k: f64 = k * w + (1.0 - k) * (1.0 - w);
                    accum += val_i * val_j * val_k * c[ui][uj][uk];
                }
            }
        }
        return accum;
    }
};
