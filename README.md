# What

Trying to learn some graphics and some [Zig](https://ziglang.org/) following the book(s) [Ray Tracing in One Weekend — The Book Series](https://raytracing.github.io).

All (almost) commits have the .ppm file the code at that point generate.

# How to run

Build (zig 0.14.1)
```bash
zig build -Doptimize=ReleaseFast
```

Run for help
```bash
./zig-out/bin/zrt 
```

Run a scene
```bash
./zig-out/bin/zrt -scene=1 > image.ppm
```

The output is a image.ppm file. Most of OS opens it with an image visualizer.

# Final render: book one
![](/images/final_render_1.jpeg)

# 1: bouncing spheres
![](/images/1_bouncing_spheres.png)

# 2: checkered spheres
![](/images/2_checkered_spheres.png)

# 3: earth
![](/images/3_earth.png)

# 4: perlin spheres
![](/images/4_perlin_spheres.png)

# 5: quads
![](/images/5_quads.png)

# 6: diffuse light
![](/images/6_diffuse_light.png)

# 7: empty cornell box
![](/images/7_empty_cornell_box.png)

# 8: cornell box
![](/images/8_cornell_box.png)

# 9: cornell box smoke
![](/images/9_cornell_box_smoke.png)
