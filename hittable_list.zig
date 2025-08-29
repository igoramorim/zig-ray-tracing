const std = @import("std");

const Hittable = @import("hittable.zig").Hittable;
const HitRecord = @import("hittable.zig").HitRecord;
const Ray = @import("ray.zig").Ray;
const Interval = @import("interval.zig").Interval;

pub const HittableList = struct {
    objects: std.ArrayList(Hittable),

    pub fn init(allocator: std.mem.Allocator) HittableList {
        return HittableList{ .objects = std.ArrayList(Hittable).init(allocator) };
    }

    pub fn clear(self: *HittableList) void {
        self.objects.deinit();
    }

    pub fn add(self: *HittableList, obj: Hittable) !void {
        try self.objects.append(obj);
    }

    pub fn hit(self: HittableList, r: Ray, ray_t: Interval, rec: *HitRecord) bool {
        var temp_rec = HitRecord{};
        var hit_anything = false;
        var closest_so_far = ray_t.max;

        for (self.objects.items) |obj| {
            if (obj.hit(r, Interval{ .min = ray_t.min, .max = closest_so_far }, &temp_rec)) {
                hit_anything = true;
                closest_so_far = temp_rec.t;
                rec.* = temp_rec;
            }
        }

        return hit_anything;
    }
};
