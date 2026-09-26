const std = @import("std");
const stderr = std.io.getStdErr().writer();

const vec3 = @import("vec3.zig");
const Point3 = vec3.Point3;
const Hittable = @import("hittable.zig").Hittable;
const HittableList = @import("hittable.zig").HittableList;
const Sphere = @import("hittable.zig").Sphere;
const material = @import("material.zig");
const Material = material.Material;
const Lambertian = material.Lambertian;
const Metal = material.Metal;
const Dielectric = material.Dielectric;

const zgui = @import("zgui");

const default_samples_per_pixel = 30;
const min_samples_per_pixel = 1;
const max_samples_per_pixel = 10_000;
var samples_per_pixel: i32 = default_samples_per_pixel;

const default_max_depth = 20;
const min_max_depth = 1;
const max_max_depth = 100;
var max_depth: i32 = default_max_depth;

const default_fov = 45.0;
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

const default_look_from = [3]f32{ 0.0, 0.0, 15.0 };
const min_look_from = -50.0;
const max_look_from = 50.0;
var look_from = default_look_from;

const default_look_at = [3]f32{ 0.0, 0.0, 0.0 };
const min_look_at = -50.0;
const max_look_at = 50.0;
var look_at = default_look_at;

var object_selected: ?usize = null;
const objects_label = [_][:0]const u8{
    "Sphere",
    "Plane",
    "Cube",
    "Constant Medium",
};

var material_selected: ?usize = null;
const materials_label = [_][:0]const u8{
    "Lambertian",
    "Metal",
    "Dielectric",
    "Diffuse Light",
    "Isotropic",
};

pub const Opts = struct {
    width: u32,
    samples_per_pixel: u32 = default_samples_per_pixel,
    max_depth: u32 = default_max_depth,
    fov: f64 = default_fov,
    defocus_angle: f64 = default_defocus_angle,
    focus_dist: f64 = default_focus_dist,
    look_from: Point3 = default_look_from,
    look_at: Point3 = default_look_at,
    world: HittableList = undefined,
};

const ObjectType = enum(u8) {
    sphere,
    plane,
    cube,
    constant_medium,
};

const MaterialType = enum(u8) {
    lambertian,
    metal,
    dielectric,
    diffuse_light,
    isotropic,
};

const default_obj_pos = [3]f32{ 0.0, 0.0, 0.0 };
const min_obj_pos = -50.0;
const max_obj_pos = 50.0;

const default_obj_radius = 2.0;
const min_obj_radius = 1.0;
const max_obj_radius = 90.0;

const default_mat_color = [3]f32{ 1.0, 0.0, 0.0 };

const default_mat_fuzz = 0.5;
const max_mat_fuzz = 1.0;
const min_mat_fuzz = 0.0;

const default_mat_refraction = 1.0;
const max_mat_refraction = 5.0;
const min_mat_refraction = 0.0;

const ObjOpts = struct {
    obj_type: ObjectType = undefined,
    obj_pos: [3]f32 = default_obj_pos,
    obj_radius: f32 = default_obj_radius,

    mat_type: MaterialType = undefined,
    mat_color: [3]f32 = default_mat_color,
    mat_fuzz: f32 = default_mat_fuzz,
    mat_refraction: f32 = default_mat_refraction,
};

pub const Editor = struct {
    allocator: std.mem.Allocator,
    render_opts: Opts,
    render_progress: *u8 = undefined,

    obj_opts: ObjOpts,
    objects: std.ArrayList(ObjOpts),

    pub fn init(allocator: std.mem.Allocator, canvas_width: u32, render_progress: *u8) Editor {
        return .{
            .allocator = allocator,
            .render_opts = .{
                .world = HittableList.init(allocator),
                .width = canvas_width,
            },
            .render_progress = render_progress,
            .obj_opts = ObjOpts{},
            .objects = std.ArrayList(ObjOpts).init(allocator),
        };
    }

    pub fn deinit(_: *Editor) void {
        // TODO: clean / deinit stuff
    }

    pub fn render(self: *Editor, width: u32, height: u32) !bool {
        zgui.backend.newFrame(width, height);
        defer zgui.backend.draw();

        if (zgui.begin("Camera", .{})) {
            defer zgui.end();

            if (zgui.sliderInt("Samples per pixel", .{
                .v = &samples_per_pixel,
                .min = min_samples_per_pixel,
                .max = max_samples_per_pixel,
            })) {
                self.render_opts.samples_per_pixel = @intCast(samples_per_pixel);
            }

            if (zgui.sliderInt("Max depth", .{
                .v = &max_depth,
                .min = min_max_depth,
                .max = max_max_depth,
            })) {
                self.render_opts.max_depth = @intCast(max_depth);
            }

            if (zgui.sliderFloat("FOV", .{
                .v = &fov,
                .min = min_fov,
                .max = max_fov,
            })) {
                self.render_opts.fov = @floatCast(fov);
            }

            if (zgui.sliderFloat("Defocus Angle", .{
                .v = &defocus_angle,
                .min = min_defocus_angle,
                .max = max_defocus_angle,
            })) {
                self.render_opts.defocus_angle = @floatCast(defocus_angle);
            }

            if (zgui.sliderFloat("Focus Distance", .{
                .v = &focus_dist,
                .min = min_focus_dist,
                .max = max_focus_dist,
            })) {
                self.render_opts.focus_dist = @floatCast(focus_dist);
            }

            if (zgui.sliderFloat3("Look From", .{
                .v = &look_from,
                .min = min_look_from,
                .max = max_look_from,
            })) {
                self.render_opts.look_from = look_from;
            }

            if (zgui.sliderFloat3("Look At", .{
                .v = &look_at,
                .min = min_look_at,
                .max = max_look_at,
            })) {
                self.render_opts.look_at = look_at;
            }

            if (zgui.button("Render", .{})) {
                self.render_opts.world.objects.clearRetainingCapacity();

                for (self.objects.items) |obj| {
                    const mat = try self.allocator.create(Material);
                    const hittable = try self.allocator.create(Hittable);

                    if (obj.mat_type == MaterialType.lambertian) {
                        const lamb = try self.allocator.create(Lambertian);
                        lamb.* = try Lambertian.init(self.allocator, obj.mat_color);
                        mat.* = lamb.mat();
                    } else if (obj.mat_type == MaterialType.metal) {
                        const metal = try self.allocator.create(Metal);
                        metal.* = Metal.init(obj.mat_color, obj.mat_fuzz);
                        mat.* = metal.mat();
                    } else if (obj.mat_type == MaterialType.dielectric) {
                        const dielectric = try self.allocator.create(Dielectric);
                        dielectric.* = Dielectric{ .refractionIndex = obj.mat_refraction };
                        mat.* = dielectric.mat();
                    }

                    if (obj.obj_type == ObjectType.sphere) {
                        const sphere = try self.allocator.create(Sphere);
                        sphere.* = Sphere.init(obj.obj_pos, obj.obj_radius, mat.*);
                        hittable.* = sphere.hittable();
                    }
                    try self.render_opts.world.add(hittable.*);
                }

                std.debug.print("render:\n", .{});
                for (self.render_opts.world.objects.items) |obj| {
                    std.debug.print("{any}\n\n", .{obj});
                }
                std.debug.print("\n", .{});

                return true;
            }

            zgui.text("{d}%", .{self.render_progress.*});
        }

        if (zgui.begin("Objects", .{})) {
            defer zgui.end();

            // Objects
            //     Sphere
            //         Center (Position)
            //         Radius
            //         Moving? Center1 / Center2
            //         Material
            //     Plane (Quad)
            //         Corner (Point)
            //         U
            //         V
            //         Material
            //     Box
            //         Corners (Point A / B)
            //         Material
            //     ConstantMedium
            //         Boundary (Hittable)
            //         Density
            //         Color / Texture
            // Materials
            //     Lambertian
            //         Color / Texture
            //     Metal
            //         Color
            //         Fuzz
            //     Dielectric
            //         Refraction
            //     DiffuseLight
            //         Color / Texture
            //     Isotropic
            //         Color / Texture
            // BVH??
            // Translate / Rotate??

            // object
            const object_label = if (object_selected != null)
                objects_label[object_selected.?]
            else
                "Choose an object";
            if (zgui.beginCombo("Object", .{ .preview_value = object_label })) {
                defer zgui.endCombo();

                for (objects_label, 0..) |label, index| {
                    const is_selected = (object_selected == index);
                    if (zgui.selectable(label, .{ .selected = is_selected })) {
                        object_selected = index;
                    }
                }
            }

            // object details
            if (object_selected != null) {
                if (object_selected.? == @intFromEnum(ObjectType.sphere)) {
                    if (zgui.sliderFloat3("Position", .{
                        .v = &self.obj_opts.obj_pos,
                        .min = min_obj_pos,
                        .max = max_obj_pos,
                    })) {}

                    if (zgui.sliderFloat("Radius", .{
                        .v = &self.obj_opts.obj_radius,
                        .min = min_obj_radius,
                        .max = max_obj_radius,
                    })) {}

                    // Moving?
                }
            }

            // material
            if (object_selected != null) {
                const material_label = if (material_selected != null)
                    materials_label[material_selected.?]
                else
                    "Choose a material";

                if (zgui.beginCombo("Material", .{ .preview_value = material_label })) {
                    defer zgui.endCombo();

                    for (materials_label, 0..) |label, index| {
                        const is_selected = (material_selected == index);
                        if (zgui.selectable(label, .{ .selected = is_selected })) {
                            material_selected = index;
                        }
                    }
                }
            }

            // material details
            if (material_selected != null) {
                if (material_selected == @intFromEnum(MaterialType.lambertian)) {
                    if (zgui.colorEdit3("Color", .{
                        .col = &self.obj_opts.mat_color,
                    })) {}
                } else if (material_selected == @intFromEnum(MaterialType.metal)) {
                    if (zgui.colorEdit3("Color", .{
                        .col = &self.obj_opts.mat_color,
                    })) {}

                    if (zgui.sliderFloat("Fuzz", .{
                        .v = &self.obj_opts.mat_fuzz,
                        .min = min_mat_fuzz,
                        .max = max_mat_fuzz,
                    })) {}
                } else if (material_selected == @intFromEnum(MaterialType.dielectric)) {
                    if (zgui.sliderFloat("Refraction", .{
                        .v = &self.obj_opts.mat_refraction,
                        .min = min_mat_refraction,
                        .max = max_mat_refraction,
                    })) {}
                }
            }

            if (object_selected != null and material_selected != null) {
                if (zgui.button("Add", .{})) {
                    self.obj_opts.obj_type = @enumFromInt(object_selected.?);
                    self.obj_opts.mat_type = @enumFromInt(material_selected.?);

                    try self.objects.append(self.obj_opts);

                    std.debug.print("add:\n", .{});
                    for (self.objects.items) |obj| {
                        std.debug.print("{any}\n\n", .{obj});
                    }
                    std.debug.print("\n", .{});
                }
            }

            if (self.objects.items.len > 0) {
                if (zgui.button("Clear", .{})) {
                    self.objects.clearRetainingCapacity();
                }
            }

            zgui.separator();

            // TODO: show the current objects to quick edits
        }

        return false;
    }
};
