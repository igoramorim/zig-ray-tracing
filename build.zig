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

    const flags_module = b.addModule(
        "flags",
        .{ .root_source_file = std.Build.LazyPath{ .cwd_relative = "internal/flags.zig" } },
    );
    exe.root_module.addImport("flags", flags_module);

    exe.addIncludePath(std.Build.LazyPath{ .cwd_relative = "external" });
    exe.linkLibC();
    exe.addCSourceFiles(.{
        .files = &[_][]const u8{"external/stb_image.c"},
        .flags = &[_][]const u8{"-g"},
    });

    b.installArtifact(exe);
}
