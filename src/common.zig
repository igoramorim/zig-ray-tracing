const std = @import("std");

pub const infinity: f64 = std.math.floatMax(f64);

pub fn deg_to_rad(degrees: f64) f64 {
    return (degrees * std.math.pi) / 180.0;
}

// returns a random real number in [0, 1)
pub fn rand_f64_01() f64 {
    var seed: u64 = undefined;
    std.posix.getrandom(std.mem.asBytes(&seed)) catch |err| {
        std.debug.print("random number: {any}\n", .{err});
    };
    var prng = std.Random.DefaultPrng.init(seed);

    const rand = prng.random();

    return rand.float(f64);
}

// returns a random real number in [min, max)
pub fn rand_f64(min: f64, max: f64) f64 {
    return min + (max - min) * rand_f64_01();
}

// returns a random integer in [min, max)
pub fn rand_i64(min: i64, max: i64) i64 {
    const fmin: f64 = @as(f64, @floatFromInt(min));
    const fmax: f64 = @as(f64, @floatFromInt(max + 1));
    const value = rand_f64(fmin, fmax);
    return @as(i64, @intFromFloat(value));
}
