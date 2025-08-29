const common = @import("common.zig");

pub const Interval = struct {
    min: f64 = common.infinity,
    max: f64 = -common.infinity,

    pub fn size(self: Interval) f64 {
        return self.max - self.min;
    }

    pub fn contains(self: Interval, x: f64) bool {
        return self.min <= x and x <= self.max;
    }

    pub fn surrounds(self: Interval, x: f64) bool {
        return self.min < x and x < self.max;
    }

    pub fn clamp(self: Interval, x: f64) f64 {
        if (x < self.min) return self.min;
        if (x > self.max) return self.max;
        return x;
    }
};

pub const empty: Interval = Interval{ .min = common.infinity, .max = -common.infinity };
pub const universe: Interval = Interval{ .min = -common.infinity, .max = common.infinity };
