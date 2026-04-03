const vec = @import("vec3.zig");
const Point3 = vec.Point3;
const Vec3 = vec.Vec3;
const f3 = vec.f3;
const common = @import("common.zig");

const point_count: i32 = 256;

pub const Perlin = struct {
    rand_vec: [point_count]Vec3 = undefined,
    perm_x: [point_count]i32 = undefined,
    perm_y: [point_count]i32 = undefined,
    perm_z: [point_count]i32 = undefined,

    pub fn init() Perlin {
        var perlin = Perlin{};

        for (0..point_count) |i| {
            perlin.rand_vec[i] = vec.unit_vector(vec.rand(-1.0, 1.0));
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

        var c: [2][2][2]Vec3 = undefined;

        for (0..2) |di| {
            for (0..2) |dj| {
                for (0..2) |dk| {
                    const idx_i: usize = @as(usize, @as(u32, @bitCast(i + @as(i32, @intCast(di))))) & 255;
                    const idx_j: usize = @as(usize, @as(u32, @bitCast(j + @as(i32, @intCast(dj))))) & 255;
                    const idx_k: usize = @as(usize, @as(u32, @bitCast(k + @as(i32, @intCast(dk))))) & 255;
                    const idx: usize = @intCast(self.perm_x[idx_i] ^ self.perm_y[idx_j] ^ self.perm_z[idx_k]);
                    c[di][dj][dk] = self.rand_vec[idx];
                }
            }
        }

        return interp(c, u, v, w);
    }

    fn interp(c: [2][2][2]Vec3, u: f64, v: f64, w: f64) f64 {
        const uu = u * u * (3 - 2 * u);
        const vv = v * v * (3 - 2 * v);
        const ww = w * w * (3 - 2 * w);
        var accum: f64 = 0;

        for (0..2) |ui| {
            for (0..2) |uj| {
                for (0..2) |uk| {
                    const i: f64 = @as(f64, @floatFromInt(ui));
                    const j: f64 = @as(f64, @floatFromInt(uj));
                    const k: f64 = @as(f64, @floatFromInt(uk));

                    const weight_v = Vec3{ u - i, v - j, w - k };

                    const val_i: f64 = i * uu + (1.0 - i) * (1.0 - uu);
                    const val_j: f64 = j * vv + (1.0 - j) * (1.0 - vv);
                    const val_k: f64 = k * ww + (1.0 - k) * (1.0 - ww);
                    const val: f64 = val_i * val_j * val_k;
                    accum += vec.dot(f3(val) * c[ui][uj][uk], weight_v);
                }
            }
        }

        return accum;
    }
};
