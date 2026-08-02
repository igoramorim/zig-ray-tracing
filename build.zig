const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});
    const optimize = b.standardOptimizeOption(.{});

    const exe = b.addExecutable(.{
        .name = "zrt",
        .root_module = b.createModule(.{
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
        }),
    });

    exe.addIncludePath(std.Build.LazyPath{ .cwd_relative = "external" });
    exe.linkLibC();
    exe.addCSourceFiles(.{
        .files = &[_][]const u8{"external/stb_image.c"},
        .flags = &[_][]const u8{"-g"},
    });

    const zglfw = b.dependency("zglfw", .{
        .target = target,
        .optimize = optimize,
    });
    exe.root_module.addImport("zglfw", zglfw.module("root"));
    exe.linkLibrary(zglfw.artifact("glfw"));

    const zopengl = b.dependency("zopengl", .{});
    exe.root_module.addImport("zopengl", zopengl.module("root"));

    const zgui = b.dependency("zgui", .{
        .target = target,
        .backend = .glfw_opengl3,
    });
    exe.root_module.addImport("zgui", zgui.module("root"));
    exe.linkLibrary(zgui.artifact("imgui"));

    b.installArtifact(exe);
}
