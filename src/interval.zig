const common = @import("common.zig");

pub const Interval = struct {
    min: f64 = common.infinity,
    max: f64 = -common.infinity,

    pub fn init_from_intervals(a: Interval, b: Interval) Interval {
        return Interval{
            .min = if (a.min <= b.min) a.min else b.min,
            .max = if (a.max >= b.max) a.max else b.max,
        };
    }

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

    pub fn empty() Interval {
        return Interval{ .min = common.infinity, .max = -common.infinity };
    }

    pub fn universe() Interval {
        return Interval{ .min = -common.infinity, .max = common.infinity };
    }

    pub fn expand(self: Interval, delta: f64) Interval {
        const padding: f64 = delta / 2;
        return Interval{ .min = self.min - padding, .max = self.max + padding };
    }
};
