const std = @import("std");
const stdout = std.io.getStdOut().writer();
const debug = std.debug;

pub fn main() !void {
    // world
    var world = HittableList.Init();
    defer world.Clear();

    const ground = Lambertian.Init(Color{ .x = 0.8, .y = 0.8, .z = 0.0 });
    var matGround = Material{ .lamertian = ground };

    const center = Lambertian.Init(Color{ .x = 0.1, .y = 0.2, .z = 0.5 });
    var matCenter = Material{ .lamertian = center };

    const left = Dielectric{ .refractionIndex = 1.5 };
    var matLeft = Material{ .dielectric = left };

    const bubble = Dielectric{ .refractionIndex = 1.0 / 1.5 };
    var matBubble = Material{ .dielectric = bubble };

    const right = Metal.Init(Color{ .x = 0.8, .y = 0.6, .z = 0.2 }, 1.0);
    var matRight = Material{ .metal = right };

    try world.Add(Sphere.Init(Point3{ .x = 0.0, .y = -100.5, .z = -1.0 }, 100.0, &matGround));
    try world.Add(Sphere.Init(Point3{ .x = 0.0, .y = 0.0, .z = -1.2 }, 0.5, &matCenter));
    try world.Add(Sphere.Init(Point3{ .x = -1.0, .y = 0.0, .z = -1.0 }, 0.5, &matLeft));
    try world.Add(Sphere.Init(Point3{ .x = -1.0, .y = 0.0, .z = -1.0 }, 0.4, &matBubble));
    try world.Add(Sphere.Init(Point3{ .x = 1.0, .y = 0.0, .z = -1.0 }, 0.5, &matRight));

    // camera
    var cam = Camera{};
    cam.aspectRadio = 16.0 / 9.0;
    cam.imageWidth = 400;
    cam.samplesPerPixel = 100;
    cam.maxDepth = 50;

    try cam.Render(world);
}

const Vec3 = struct {
    x: f64 = 0.0,
    y: f64 = 0.0,
    z: f64 = 0.0,

    pub fn X(self: Vec3) f64 {
        return self.x;
    }

    pub fn Y(self: Vec3) f64 {
        return self.y;
    }

    pub fn Z(self: Vec3) f64 {
        return self.z;
    }

    pub fn Length(self: Vec3) f64 {
        return std.math.sqrt(self.LengthSquared());
    }

    pub fn LengthSquared(self: Vec3) f64 {
        return (self.x * self.x) + (self.y * self.y) + (self.z * self.z);
    }

    pub fn DivI64(self: Vec3, i: i64) Vec3 {
        const value: f64 = 1 / @as(f64, @floatFromInt(i));
        return self.MultF64(value);
    }

    pub fn DivF64(self: Vec3, f: f64) Vec3 {
        const value: f64 = 1 / f;
        return self.MultF64(value);
    }

    pub fn Mult(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x * v.x,
            .y = self.y * v.y,
            .z = self.z * v.z,
        };
    }

    pub fn MultI64(self: Vec3, i: i64) Vec3 {
        return Vec3{
            .x = self.x * @as(f64, @floatFromInt(i)),
            .y = self.y * @as(f64, @floatFromInt(i)),
            .z = self.z * @as(f64, @floatFromInt(i)),
        };
    }

    pub fn MultF64(self: Vec3, f: f64) Vec3 {
        return Vec3{
            .x = f * self.x,
            .y = f * self.y,
            .z = f * self.z,
        };
    }

    pub fn Sub(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x - v.x,
            .y = self.y - v.y,
            .z = self.z - v.z,
        };
    }

    pub fn Add(self: Vec3, v: Vec3) Vec3 {
        return Vec3{
            .x = self.x + v.x,
            .y = self.y + v.y,
            .z = self.z + v.z,
        };
    }

    pub fn AddF64(self: Vec3, f: f64) Vec3 {
        return Vec3{
            .x = self.x + f,
            .y = self.y + f,
            .z = self.z + f,
        };
    }

    pub fn Reverse(self: Vec3) Vec3 {
        return Vec3{
            .x = -self.x,
            .y = -self.y,
            .z = -self.z,
        };
    }

    pub fn Random_01() Vec3 {
        return Vec3{
            .x = rand_f64_01(),
            .y = rand_f64_01(),
            .z = rand_f64_01(),
        };
    }

    pub fn Random(min: f64, max: f64) Vec3 {
        return Vec3{
            .x = rand_f64(min, max),
            .y = rand_f64(min, max),
            .z = rand_f64(min, max),
        };
    }

    // return true if the vector is close to zero in all dimensions
    pub fn NearZero(self: Vec3) bool {
        const s: f64 = 1e-8;
        return @abs(self.x) < s and @abs(self.y) < s and @abs(self.z) < s;
    }
};

const Point3 = Vec3;

const Color = Vec3;

fn writeColor(pixelColor: Color) !void {
    // values in range 0.0 to 1.0
    var r: f64 = pixelColor.X();
    var g: f64 = pixelColor.Y();
    var b: f64 = pixelColor.Z();

    r = linearToGamma(r);
    g = linearToGamma(g);
    b = linearToGamma(b);

    // values in range 0 to 255
    const intensity = Interval.Init(0.000, 0.999);
    const rbyte: i64 = @as(i64, @intFromFloat(256 * intensity.Clamp(r)));
    const gbyte: i64 = @as(i64, @intFromFloat(256 * intensity.Clamp(g)));
    const bbyte: i64 = @as(i64, @intFromFloat(256 * intensity.Clamp(b)));

    try stdout.print("{d} {d} {d}\n", .{ rbyte, gbyte, bbyte });
}

fn linearToGamma(linearComponent: f64) f64 {
    if (linearComponent > 0) {
        return std.math.sqrt(linearComponent);
    }
    return 0;
}

fn hitSphere(center: Vec3, radius: f64, ray: Ray) f64 {
    const oc: Vec3 = center.Sub(ray.Origin());
    const a: f64 = ray.Direction().LengthSquared();
    const h: f64 = dot(ray.Direction(), oc);
    const c: f64 = oc.LengthSquared() - (radius * radius);
    const discriminant: f64 = (h * h) - (a * c);

    if (discriminant < 0.0) {
        return -1.0;
    }

    return (h - std.math.sqrt(discriminant)) / a;
}

fn dot(u: Vec3, v: Vec3) f64 {
    return u.X() * v.X() +
        u.Y() * v.Y() +
        u.Z() * v.Z();
}

fn unitVector(v: Vec3) Vec3 {
    return v.DivF64(v.Length());
}

fn randomUnitVector() Vec3 {
    while (true) {
        const p = Vec3.Random(-1.0, 1.0);
        const lensq = p.LengthSquared();
        // NOTE: in the tutorial there is another check here
        if (lensq < 1.0) {
            return p.DivF64(std.math.sqrt(lensq));
        }
    }
}

fn randomOnHemisphere(normal: Vec3) Vec3 {
    const onUnitSphere = randomUnitVector();
    if (dot(onUnitSphere, normal) > 0.0) { // in the same hemisphere as the normal
        return onUnitSphere;
    }
    return onUnitSphere.Reverse();
}

fn reflect(v: Vec3, n: Vec3) Vec3 {
    const vxn = dot(v, n);
    return v.Sub(n.MultF64(vxn).MultF64(2));
}

fn refract(uv: Vec3, n: Vec3, etai_over_etat: f64) Vec3 {
    const cost_theta = @min(dot(uv.Reverse(), n), 1.0);
    const r_out_perp = n.MultF64(cost_theta).Add(uv).MultF64(etai_over_etat);
    const r_out_parallel = n.MultF64(-@sqrt((@abs(1.0 - r_out_perp.LengthSquared()))));
    return r_out_perp.Add(r_out_parallel);
}

const Camera = struct {
    aspectRadio: f64 = 1.0, // ratio of image width over height
    imageWidth: i64 = 100, // rendered image width in pixel count
    imageHeight: i64 = undefined, // rendered image height
    cameraCenter: Point3 = Point3{}, // camera center
    pixel00Loc: Point3 = Point3{}, // location of pixel 0,0
    pixelDeltaU: Vec3 = Vec3{}, // offset to pixel to the right
    pixelDeltaV: Vec3 = Vec3{}, // off to pixel below
    samplesPerPixel: i64 = 10, // count of random samples for each pixel
    pixelSamplesScale: f64 = undefined, // color scale factor for a sum of pixel samples
    maxDepth: i64 = 10, // maximum number of rays to bounce into scene

    pub fn Render(self: *Camera, world: HittableList) !void {
        self.Initialize();

        try stdout.print("P3\n{d} {d}\n255\n", .{ self.imageWidth, self.imageHeight });

        var j: i64 = 0;
        while (j < self.imageHeight) : (j = j + 1) {
            debug.print("scan lines remaining: {d}\n", .{(self.imageHeight - j)});

            var i: i64 = 0;
            while (i < self.imageWidth) : (i = i + 1) {
                var pixelColor = Color{};
                var sample: i64 = 0;

                while (sample < self.samplesPerPixel) : (sample = sample + 1) {
                    const ray = self.getRay(i, j);
                    pixelColor = pixelColor.Add(self.rayColor(ray, self.maxDepth, world));
                }

                try writeColor(pixelColor.MultF64(self.pixelSamplesScale));
            }
        }

        debug.print("done!\n", .{});
    }

    fn Initialize(self: *Camera) void {
        const heightf: f64 = @as(f64, @floatFromInt(self.imageWidth)) / self.aspectRadio;
        self.imageHeight = @as(i64, @intFromFloat(heightf));
        if (self.imageHeight < 1) {
            self.imageHeight = 1;
        }
        debug.print("width: {any} aspect radio: {any} height: {any}\n", .{ self.imageWidth, self.aspectRadio, self.imageHeight });

        self.pixelSamplesScale = 1.0 / @as(f64, @floatFromInt(self.samplesPerPixel));

        // determine viewport dimensions
        const focalLength: f64 = 1.0;
        const viewportHeight: f64 = 2.0;
        const viewportWidth: f64 = viewportHeight * (@as(f64, @floatFromInt(self.imageWidth)) / @as(f64, @floatFromInt(self.imageHeight)));

        // calculate the vectors across the horizontal and down the vertical viewport edges
        const viewportU = Vec3{ .x = viewportWidth };
        const viewportV = Vec3{ .y = -viewportHeight };

        // calculate the horizontal and vertical delta vectors from pixel to pixel
        self.pixelDeltaU = viewportU.DivI64(self.imageWidth);
        self.pixelDeltaV = viewportV.DivI64(self.imageHeight);

        // calculate the location of the upper left pixel
        const viewportUpperLeft = self.cameraCenter.Sub(Vec3{ .z = focalLength }).Sub(viewportU.DivI64(2)).Sub(viewportV.DivI64(2));
        self.pixel00Loc = self.pixelDeltaU.Add(self.pixelDeltaV).MultF64(0.5).Add(viewportUpperLeft);
    }

    // construct a camera ray originating from the origin and directed at randomdly sampled
    // point around the pixel location i, j
    fn getRay(self: Camera, i: i64, j: i64) Ray {
        const offset = self.sampleSquare();

        const ixu = self.pixelDeltaU.MultF64((@as(f64, @floatFromInt(i)) + offset.X()));
        const jyv = self.pixelDeltaV.MultF64((@as(f64, @floatFromInt(j)) + offset.Y()));
        const pixelSample = self.pixel00Loc.Add(ixu).Add(jyv);

        const rayOrigin = self.cameraCenter;
        const rayDirection = pixelSample.Sub(rayOrigin);

        return Ray{ .orig = rayOrigin, .dir = rayDirection };
    }

    // returns the vector to a random point in the [-.5, -.5]-[+.5, +.5] unit square
    fn sampleSquare(_: Camera) Vec3 {
        return Vec3{
            .x = rand_f64_01() - 0.5,
            .y = rand_f64_01() - 0.5,
        };
    }

    fn rayColor(self: Camera, ray: Ray, depth: i64, world: HittableList) Color {
        // if we've exceeded the ray bounce limit, no more light is gathered
        if (depth <= 0) {
            return Color{};
        }

        var rec = HitRecord{};
        if (world.Hit(ray, Interval.Init(0.001, infinity), &rec)) {
            var scattered: Ray = undefined;
            var attenuation = Color{};

            if (rec.mat.Scatter(ray, rec, &attenuation, &scattered)) {
                return self.rayColor(scattered, depth - 1, world).Mult(attenuation);
            }

            return Color{};
        }

        const unitDirection = unitVector(ray.Direction());
        const a: f64 = 0.5 * (unitDirection.Y() + 1.0);

        const white = Color{ .x = 1.0, .y = 1.0, .z = 1.0 };
        const blue = Color{ .x = 0.5, .y = 0.7, .z = 1.0 };

        return white.MultF64(1.0 - a).Add(blue.MultF64(a));
    }
};

const Ray = struct {
    orig: Point3,
    dir: Vec3,

    pub fn Origin(self: Ray) Point3 {
        return self.orig;
    }

    pub fn Direction(self: Ray) Vec3 {
        return self.dir;
    }

    pub fn At(self: Ray, t: f64) Point3 {
        return self.dir.MultF64(t).Add(self.orig);
    }
};

const HitRecord = struct {
    p: Point3 = Point3{},
    normal: Vec3 = Vec3{},
    mat: *Material = undefined,
    t: f64 = undefined,
    frontFace: bool = undefined,

    pub fn SetFaceNormal(self: *HitRecord, ray: Ray, outwardNormal: Vec3) void {
        // sets the hit record normal vector
        // NOTE: the parameter 'outwardNormal' is assumed to have unit length

        self.frontFace = dot(ray.Direction(), outwardNormal) < 0.0;
        if (self.frontFace) {
            self.normal = outwardNormal;
        } else {
            self.normal = outwardNormal.Reverse();
        }
    }

    pub fn Normal(self: HitRecord) Vec3 {
        return self.normal;
    }
};

const Sphere = struct {
    center: Point3,
    radius: f64,
    mat: *Material = undefined,

    pub fn Init(center: Point3, radius: f64, mat: *Material) Sphere {
        return Sphere{
            .center = center,
            .radius = @max(0.0, radius),
            .mat = mat,
        };
    }

    pub fn Hit(self: Sphere, ray: Ray, rayT: Interval, rec: *HitRecord) bool {
        const oc: Vec3 = self.center.Sub(ray.Origin());
        const a: f64 = ray.Direction().LengthSquared();
        const h: f64 = dot(ray.Direction(), oc);
        const c: f64 = oc.LengthSquared() - (self.radius * self.radius);

        const discriminant: f64 = (h * h) - (a * c);
        if (discriminant < 0.0) {
            return false;
        }

        const sqrtd: f64 = std.math.sqrt(discriminant);

        // find the nearest root tha lies in the acceptable range
        var root: f64 = (h - sqrtd) / a;
        if (!rayT.Surrounds(root)) {
            root = (h + sqrtd) / a;
            if (!rayT.Surrounds(root)) {
                return false;
            }
        }

        rec.t = root;
        rec.p = ray.At(rec.t);
        const outwardNormal: Vec3 = rec.p.Sub(self.center).DivF64(self.radius);
        rec.SetFaceNormal(ray, outwardNormal);
        rec.mat = self.mat;

        return true;
    }
};

const HittableList = struct {
    objects: std.ArrayList(Sphere),

    pub fn Init() HittableList {
        // TODO: Not sure about the allocator type
        return HittableList{ .objects = std.ArrayList(Sphere).init(std.heap.page_allocator) };
    }

    pub fn Clear(self: *HittableList) void {
        // TODO: Check if this is the way
        self.objects.deinit();
    }

    pub fn Add(self: *HittableList, obj: Sphere) !void {
        try self.objects.append(obj);
    }

    pub fn Hit(self: HittableList, ray: Ray, rayT: Interval, rec: *HitRecord) bool {
        var tempRec = HitRecord{};
        var hitAnything = false;
        var closestSoFar = rayT.max;

        // @compileLog(@TypeOf(rec));
        // @compileLog(@TypeOf(tempRec));

        for (self.objects.items) |obj| {
            if (obj.Hit(ray, Interval.Init(rayT.min, closestSoFar), &tempRec)) {
                hitAnything = true;
                closestSoFar = tempRec.t;
                rec.* = tempRec;
            }
        }

        return hitAnything;
    }
};

const Material = union(enum) {
    lamertian: Lambertian,
    metal: Metal,
    dielectric: Dielectric,

    pub fn Scatter(self: Material, rayIn: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        switch (self) {
            inline else => |mat| return mat.Scatter(rayIn, rec, attenuation, scattered),
        }
    }
};

const Lambertian = struct {
    albedo: Color,

    pub fn Init(albedo: Color) Lambertian {
        return Lambertian{ .albedo = albedo };
    }

    pub fn Scatter(self: Lambertian, _: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        var scatterDirection = rec.Normal().Add(randomUnitVector());

        // catch degenerate scatter direction
        if (scatterDirection.NearZero()) {
            scatterDirection = rec.Normal();
        }

        scattered.* = Ray{ .orig = rec.p, .dir = scatterDirection };
        attenuation.* = self.albedo;
        return true;
    }
};

const Metal = struct {
    albedo: Color,
    fuzz: f64,

    pub fn Init(albedo: Color, fuzz: f64) Metal {
        return Metal{ .albedo = albedo, .fuzz = if (fuzz > 1.0) 1.0 else fuzz };
    }

    pub fn Scatter(self: Metal, rayIn: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        var reflected = reflect(rayIn.Direction(), rec.Normal());
        reflected = randomUnitVector().MultF64(self.fuzz).Add(unitVector(reflected));
        scattered.* = Ray{ .orig = rec.p, .dir = reflected };
        attenuation.* = self.albedo;
        return (dot(scattered.Direction(), rec.Normal()) > 0);
    }
};

const Dielectric = struct {
    // refractive index in vacuum or air, or the ratio of the material's refractive index over
    // the refractive index of the enclosing media
    refractionIndex: f64,

    pub fn Scatter(self: Dielectric, rayIn: Ray, rec: HitRecord, attenuation: *Color, scattered: *Ray) bool {
        // attenuation 1 means the glass surface absorbs nothing
        attenuation.* = Color{ .x = 1.0, .y = 1.0, .z = 1.0 };
        const ri = if (rec.frontFace) (1.0 / self.refractionIndex) else self.refractionIndex;

        const unitDirection = unitVector(rayIn.Direction());
        const cosTheta = @min(dot(unitDirection.Reverse(), rec.Normal()), 1.0);
        const sinTheta = @sqrt(1.0 - (cosTheta * cosTheta));

        const cannotRefract = (ri * sinTheta) > 1.0;
        var direction: Vec3 = undefined;

        if (cannotRefract or reflectance(cosTheta, ri) > rand_f64_01()) {
            direction = reflect(unitDirection, rec.Normal());
        } else {
            direction = refract(unitDirection, rec.Normal(), ri);
        }

        scattered.* = Ray{ .orig = rec.p, .dir = direction };
        return true;
    }

    fn reflectance(cosine: f64, refractionIndex: f64) f64 {
        // use Schlick's approximation for reflectance
        var r0 = (1 - refractionIndex) / (1 + refractionIndex);
        r0 = r0 * r0;
        return r0 + (1 - r0) * std.math.pow(f64, (1 - cosine), 5);
    }
};

const Interval = struct {
    min: f64 = infinity,
    max: f64 = -infinity,

    pub fn Init(min: f64, max: f64) Interval {
        return Interval{ .min = min, .max = max };
    }

    pub fn Size(self: Interval) f64 {
        return self.max - self.min;
    }

    pub fn Contains(self: Interval, x: f64) bool {
        return self.min <= x and x <= self.max;
    }

    pub fn Surrounds(self: Interval, x: f64) bool {
        return self.min < x and x < self.max;
    }

    pub fn Clamp(self: Interval, x: f64) f64 {
        if (x < self.min) return self.min;
        if (x > self.max) return self.max;
        return x;
    }
};

const empty: Interval = Interval.Init(infinity, -infinity);
const universe: Interval = Interval.Init(-infinity, infinity);

const infinity: f64 = std.math.floatMax(f64);
const pi: f64 = 3.1415926535897932385;

fn degToRad(degrees: f64) f64 {
    return (degrees * pi) / 180.0;
}

// returns a random real number in [0, 1)
fn rand_f64_01() f64 {
    var seed: u64 = undefined;
    std.posix.getrandom(std.mem.asBytes(&seed)) catch |err| {
        std.debug.print("random number: {any}\n", .{err});
    };
    var prng = std.Random.DefaultPrng.init(seed);

    const rand = prng.random();

    return rand.float(f64);
}

// returns a random real number in [min, max)
fn rand_f64(min: f64, max: f64) f64 {
    return min + (max - min) * rand_f64_01();
}
