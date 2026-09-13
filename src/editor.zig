const std = @import("std");
const stderr = std.io.getStdErr().writer();

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;

const zgui = @import("zgui");

const default_samples_per_pixel = 30;
const min_samples_per_pixel = 1;
const max_samples_per_pixel = 10_000;
var samples_per_pixel: i32 = default_samples_per_pixel;

const default_max_depth = 20;
const min_max_depth = 1;
const max_max_depth = 100;
var max_depth: i32 = default_max_depth;

const default_fov = 20.0;
const min_fov = 1.0;
const max_fov = 90.0;
var fov: f32 = default_fov;

const default_defocus_angle = 0.0;
const min_defocus_angle = 0.0;
const max_defocus_angle = 10.0;
var defocus_angle: f32 = default_defocus_angle;

const default_focus_dist = 10.0;
const min_focus_dist = 1.0;
const max_focus_dist = 100.0;
var focus_dist: f32 = default_focus_dist;

const default_look_from = [3]f32{ 0.0, 2.0, 3.0 };
const min_look_from = -50.0;
const max_look_from = 50.0;
var look_from = default_look_from;

const default_look_at = [3]f32{ 0.0, 0.0, 0.0 };
const min_look_at = -50.0;
const max_look_at = 50.0;
var look_at = default_look_at;

pub const Opts = struct {
    width: u32,
    samples_per_pixel: u32 = default_samples_per_pixel,
    max_depth: u32 = default_max_depth,
    fov: f64 = default_fov,
    defocus_angle: f64 = default_defocus_angle,
    focus_dist: f64 = default_focus_dist,
    look_from: Point3 = default_look_from,
    look_at: Point3 = default_look_at,
};

pub const Editor = struct {
    opts: Opts,
    render_progress: *u8 = undefined,

    pub fn init(canvas_width: u32, render_progress: *u8) Editor {
        return .{
            .opts = .{ .width = canvas_width },
            .render_progress = render_progress,
        };
    }

    pub fn render(self: *Editor, width: u32, height: u32) bool {
        zgui.backend.newFrame(width, height);
        defer {
            zgui.end();
            zgui.backend.draw();
        }

        if (zgui.begin("Editor", .{})) {
            if (zgui.sliderInt("Samples per pixel", .{
                .v = &samples_per_pixel,
                .min = min_samples_per_pixel,
                .max = max_samples_per_pixel,
            })) {
                self.opts.samples_per_pixel = @intCast(samples_per_pixel);
            }

            if (zgui.sliderInt("Max depth", .{
                .v = &max_depth,
                .min = min_max_depth,
                .max = max_max_depth,
            })) {
                self.opts.max_depth = @intCast(max_depth);
            }

            if (zgui.sliderFloat("FOV", .{
                .v = &fov,
                .min = min_fov,
                .max = max_fov,
            })) {
                self.opts.fov = @floatCast(fov);
            }

            if (zgui.sliderFloat("Defocus Angle", .{
                .v = &defocus_angle,
                .min = min_defocus_angle,
                .max = max_defocus_angle,
            })) {
                self.opts.defocus_angle = @floatCast(defocus_angle);
            }

            if (zgui.sliderFloat("Focus Distance", .{
                .v = &focus_dist,
                .min = min_focus_dist,
                .max = max_focus_dist,
            })) {
                self.opts.focus_dist = @floatCast(focus_dist);
            }

            if (zgui.sliderFloat3("Look From", .{
                .v = &look_from,
                .min = min_look_from,
                .max = max_look_from,
            })) {
                self.opts.look_from = look_from;
            }

            if (zgui.sliderFloat3("Look At", .{
                .v = &look_at,
                .min = min_look_at,
                .max = max_look_at,
            })) {
                self.opts.look_at = look_at;
            }

            if (zgui.button("Render", .{})) {
                return true;
            }

            zgui.text("{d}%", .{self.render_progress.*});
        }

        return false;
    }
};
