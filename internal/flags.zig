const std = @import("std");
const ArgIterator = std.process.ArgIterator;
const assert = std.debug.assert;

const FlagError = error{
    FlagNotFound,
    NoValueProvided,
    InvalidValue,
};

/// parses flags like -key=value --key=value -boolflag -boolflag=false
pub fn parse(comptime T: type, args: *ArgIterator, target: []const u8) FlagError!T {
    while (args.next()) |arg| {
        const is_key = if (arg[0] == '-') true else false;
        if (!is_key) {
            continue;
        }

        const name = arg_name(arg);
        if (!std.mem.eql(u8, target, name)) {
            continue;
        }

        const value = arg_value(T, arg);

        return parse_value(T, value);
    }

    return error.FlagNotFound;
}

fn arg_name(arg: []const u8) []const u8 {
    assert(arg.len > 1);
    assert(arg[0] == '-');

    var from: usize = 1;
    var to: usize = arg.len;

    if (arg[1] == '-') {
        assert(arg.len > 2);
        from = 2;
    }

    if (std.mem.indexOf(u8, arg, "=")) |i| {
        to = i;
    }

    return arg[from..to];
}

fn arg_value(comptime T: type, arg: []const u8) []const u8 {
    const split_at = std.mem.indexOf(u8, arg, "=");

    // bool flags without '=' means true, like '-key'
    if (T == bool and split_at == null) {
        return "true";
    }

    assert(arg.len > split_at.? + 1);
    return arg[split_at.? + 1 ..];
}

fn parse_value(comptime T: type, value: []const u8) FlagError!T {
    assert(value.len > 0);

    if (T == bool) {
        if (std.mem.eql(u8, value, "true")) {
            return true;
        } else if (std.mem.eql(u8, value, "false")) {
            return false;
        } else {
            return error.InvalidValue;
        }
    }

    if (@typeInfo(T) == .int) {
        return std.fmt.parseInt(T, value, 10) catch {
            return error.InvalidValue;
        };
    }

    if (@typeInfo(T) == .float) {
        return std.fmt.parseFloat(T, value) catch {
            return error.InvalidValue;
        };
    }

    if (T == []const u8 or T == [:0]const u8) return value;
    comptime unreachable;
}
