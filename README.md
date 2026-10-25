# What

Trying to learn some graphics and some [Zig](https://ziglang.org/) following the book(s) [Ray Tracing in One Weekend — The Book Series](https://raytracing.github.io).

# How to run

Build (zig 0.14.1)
```bash
zig build -Doptimize=ReleaseFast
```

Run
```bash
./zig-out/bin/zrt
```
![](/images/editor.png)

## TODO
- [] Add another type of light
- [] Cast shadows
- [] Rotation in X and Z
- [] Export render as image
- [] Load .obj models
- [] Live render (remove the Render button)
