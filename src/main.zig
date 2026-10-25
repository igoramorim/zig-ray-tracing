const std = @import("std");

const vec3 = @import("vec3.zig");
const Vec3 = vec3.Vec3;
const Point3 = vec3.Point3;
const common = @import("common.zig");
const color = @import("color.zig");
const Color = color.Color;
const interval = @import("interval.zig");
const Interval = interval.Interval;
const Hittable = @import("hittable.zig").Hittable;
const Sphere = @import("hittable.zig").Sphere;
const material = @import("material.zig");
const Lambertian = material.Lambertian;
const Metal = material.Metal;
const Dielectric = material.Dielectric;
const Ray = @import("ray.zig").Ray;
const HitRecord = @import("hittable.zig").HitRecord;
const HittableList = @import("hittable.zig").HittableList;
const Camera = @import("camera.zig").Camera;
const BVHNode = @import("hittable.zig").BVHNode;
const texture = @import("texture.zig");
const Checker = texture.Checker;
const Image = texture.Image;
const Noise = texture.Noise;
const Quad = @import("hittable.zig").Quad;
const DiffuseLight = @import("material.zig").DiffuseLight;
const Box = @import("hittable.zig").Box;
const RotateY = @import("hittable.zig").RotateY;
const Translate = @import("hittable.zig").Translate;
const ConstantMedium = @import("hittable.zig").ConstantMedium;
const Editor = @import("editor.zig").Editor;
const Opts = @import("editor.zig").Opts;

const zglfw = @import("zglfw");
const zgui = @import("zgui");
const zopengl = @import("zopengl");

const window_width = 1280;
const window_height = 960;

const canvas_width = 800;
const canvas_height = 800;

pub fn main() !void {
    try zglfw.init();
    defer zglfw.terminate();

    const gl_major = 3;
    const gl_minor = 3;
    zglfw.windowHint(.context_version_major, gl_major);
    zglfw.windowHint(.context_version_minor, gl_minor);
    zglfw.windowHint(.opengl_profile, .opengl_core_profile);
    zglfw.windowHint(.opengl_forward_compat, true);
    zglfw.windowHint(.client_api, .opengl_api);
    zglfw.windowHint(.resizable, false);

    const window = try zglfw.Window.create(window_width, window_height, "zig ray tracer", null, null);
    defer window.destroy();

    zglfw.makeContextCurrent(window);
    zglfw.swapInterval(1);

    try zopengl.loadCoreProfile(zglfw.getProcAddress, gl_major, gl_minor);
    const gl = zopengl.bindings;

    var gpa = std.heap.GeneralPurposeAllocator(.{}){};
    const gpa_allocator = gpa.allocator();
    defer _ = gpa.deinit();

    var arena = std.heap.ArenaAllocator.init(gpa_allocator);
    defer arena.deinit();
    const allocator = arena.allocator();

    zgui.init(allocator);
    defer zgui.deinit();

    zgui.backend.init(window);
    defer zgui.backend.deinit();

    const total_pixels: usize = canvas_width * canvas_height * 3; // RGB
    var state = try State.init(allocator, total_pixels);
    defer state.deinit();
    var render_thread: ?std.Thread = null;
    defer {
        // make sure to cancel the render if the app is closed while rendering
        if (render_thread) |thread| {
            state.cancel_render.store(true, .monotonic);
            thread.join();
        }
    }

    var editor = Editor.init(allocator, canvas_width, &state.progress);
    defer editor.deinit();

    var tex_id: gl.Uint = 0;
    gl.genTextures(1, &tex_id);
    gl.bindTexture(gl.TEXTURE_2D, tex_id);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST);
    gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
    gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGB, canvas_width, canvas_height, 0, gl.RGB, gl.UNSIGNED_BYTE, null);
    gl.bindTexture(gl.TEXTURE_2D, 0);

    var framebuf_id: gl.Uint = 0;
    gl.genFramebuffers(1, &framebuf_id);
    gl.bindFramebuffer(gl.READ_FRAMEBUFFER, framebuf_id);
    gl.framebufferTexture2D(gl.READ_FRAMEBUFFER, gl.COLOR_ATTACHMENT0, gl.TEXTURE_2D, tex_id, 0);
    gl.bindFramebuffer(gl.READ_FRAMEBUFFER, 0);

    while (!window.shouldClose() and window.getKey(.escape) != .press) {
        zglfw.pollEvents();

        gl.clearColor(0, 0, 0, 1);
        gl.clear(gl.COLOR_BUFFER_BIT);

        gl.bindTexture(gl.TEXTURE_2D, tex_id);
        gl.texSubImage2D(gl.TEXTURE_2D, 0, 0, 0, canvas_width, canvas_height, gl.RGB, gl.UNSIGNED_BYTE, state.buffer.ptr);

        gl.bindFramebuffer(gl.READ_FRAMEBUFFER, framebuf_id);
        gl.bindFramebuffer(gl.DRAW_FRAMEBUFFER, 0);
        // destination height and width are swapped so OpenGL does not render upside down
        gl.blitFramebuffer(0, 0, canvas_width, canvas_height, 0, canvas_height, canvas_width, 0, gl.COLOR_BUFFER_BIT, gl.NEAREST);

        const fb_size = window.getFramebufferSize();
        if (try editor.render(@intCast(fb_size[0]), @intCast(fb_size[1]))) {
            state.cancel_render.store(true, .monotonic); // cancels the current render

            if (render_thread) |thread| { // wait the thread finishes
                thread.join();
                render_thread = null;
            }

            @memset(state.buffer, 0); // clear the buffer
            state.cancel_render.store(false, .monotonic); // reset the cancel flag

            // start a new render
            state.progress = 0;
            render_thread = try std.Thread.spawn(
                .{},
                scene_runner,
                .{ allocator, editor.render_opts, &state },
            );
        }

        window.swapBuffers();
    }
}

pub const State = struct {
    allocator: std.mem.Allocator,
    buffer: []u8,
    cancel_render: std.atomic.Value(bool),
    progress: u8 = 0,

    pub fn init(allocator: std.mem.Allocator, total_pixels: usize) !State {
        const buffer = try allocator.alloc(u8, total_pixels);
        @memset(buffer, 0); // clear the buffer

        return .{
            .allocator = allocator,
            .buffer = buffer,
            .cancel_render = std.atomic.Value(bool).init(false),
        };
    }

    pub fn deinit(self: *State) void {
        self.allocator.free(self.buffer);
    }
};

fn scene_runner(allocator: std.mem.Allocator, opts: Opts, state: *State) anyerror!void {
    var cam = Camera{};
    cam.aspect_radio = 16.0 / 9.0;
    cam.image_width = opts.width;
    cam.samples_per_pixel = opts.samples_per_pixel;
    cam.max_depth = opts.max_depth;
    cam.vfov = opts.fov;
    cam.look_from = opts.look_from;
    cam.look_at = opts.look_at;
    cam.vup = Vec3{ 0.0, 1.0, 0.0 };
    cam.defocus_angle = opts.defocus_angle;
    cam.focus_dist = opts.focus_dist;
    cam.background_color = opts.background_color;
    try cam.render(allocator, opts.world.hittable(), state);
}
