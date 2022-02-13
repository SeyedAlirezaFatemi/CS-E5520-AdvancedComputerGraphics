#include "Renderer.hpp"

#include <atomic>
#include <chrono>

#include "RayTracer.hpp"

namespace FW {

Renderer::Renderer() {
    m_aoRayLength = 0.5f;
    m_aoNumRays = 16;
    m_aaNumRays = 1;
    m_raysPerSecond = 0.0f;
}

Renderer::~Renderer() {}

void Renderer::gatherLightTriangles(RayTracer* rt) {
    // gather light triangles into vector for possible later use in the area
    // light extra
    m_lightTriangles.clear();
    m_combinedLightArea = .0f;
    for (auto& tri : *rt->m_triangles) {
        MeshBase::Material* mat = tri.m_material;
        if (mat->emission.length() > .0f) {
            m_lightTriangles.push_back(&tri);
            m_combinedLightArea += tri.area();
        }
    }
}

timingResult Renderer::rayTracePicture(RayTracer* rt,
                                       Image* image,
                                       const CameraControls& cameraCtrl,
                                       ShadingMode mode) {
    // measure time to render
    LARGE_INTEGER start, stop, frequency;
    QueryPerformanceFrequency(&frequency);
    QueryPerformanceCounter(&start);  // Start time stamp
    rt->resetRayCounter();

    // this has a side effect of forcing Image to reserve its memory immediately
    // otherwise we get a rendering bug & memory leak in OpenMP parallel code
    image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection =
        Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) *
        cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // progress counter
    std::atomic<int> lines_done = 0;

    int height = image->getSize().y;
    int width = image->getSize().x;

    for (int j = 0; j < height; j++)
        for (int i = 0; i < width; i++)
            image->setVec4f(Vec2i(i, j), Vec4f(.0f));  // initialize image to 0

    const float subpixelSize = 1.0f / (m_aaNumRays * 2.0f);
// YOUR CODE HERE(R5):
// remove this to enable multithreading (you also need to enable it in the
// project properties: C++/Language/Open MP support)
#pragma omp parallel for
    for (int j = 0; j < height; ++j) {
        // Each thread must have its own random generator
        Random rnd;

        for (int i = 0; i < width; ++i) {
            Vec4f color(0, 0, 0, 0);
            for (size_t rayInPixelIdx = 0; rayInPixelIdx < m_aaNumRays;
                 rayInPixelIdx++) {
                float xJitter = rnd.getF32(-subpixelSize, subpixelSize);
                float yJitter = rnd.getF32(-subpixelSize, subpixelSize);
                // generate ray through pixel
                float jump = (rayInPixelIdx * 2 + 1) * subpixelSize;
                float x =
                    (i + jump + xJitter) / image->getSize().x * 2.0f - 1.0f;
                float y =
                    (j + jump + yJitter) / image->getSize().y * -2.0f + 1.0f;
                // point on front plane in homogeneous coordinates
                Vec4f P0(x, y, 0.0f, 1.0f);
                // point on back plane in homogeneous coordinates
                Vec4f P1(x, y, 1.0f, 1.0f);

                // apply inverse projection, divide by w to get object-space
                // points
                Vec4f Roh = (invP * P0);
                Vec3f Ro = (Roh * (1.0f / Roh.w)).getXYZ();
                Vec4f Rdh = (invP * P1);
                Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();

                // Subtract front plane point from back plane point,
                // yields ray direction.
                // NOTE that it's not normalized; the direction Rd is defined
                // so that the segment to be traced is [Ro, Ro+Rd], i.e.,
                // intersections that come _after_ the point Ro+Rd are to be
                // discarded.
                Rd = Rd - Ro;

                // trace!
                RaycastResult hit = rt->raycast(Ro, Rd);

                // if we hit something, fetch a color and insert into image

                if (hit.tri != nullptr) {
                    switch (mode) {
                        case ShadingMode_Headlight:
                            color += computeShadingHeadlight(hit, cameraCtrl);
                            break;
                        case ShadingMode_AmbientOcclusion:
                            color += computeShadingAmbientOcclusion(
                                rt, hit, cameraCtrl, rnd);
                            break;
                        case ShadingMode_Whitted:
                            color += computeShadingWhitted(
                                rt, hit, cameraCtrl, rnd, 0);
                            break;
                    }
                }
            }
            // put pixel.
            image->setVec4f(Vec2i(i, j), color / color.get(3));
        }

        // Print progress info
        ++lines_done;
        ::printf("%.2f%% \r", lines_done * 100.0f / height);
    }

    // how fast did we go?
    timingResult result;

    QueryPerformanceCounter(&stop);  // Stop time stamp

    result.duration =
        (int)((stop.QuadPart - start.QuadPart) * 1000.0 /
              frequency.QuadPart);  // Get timer result in milliseconds

    // calculate average rays per second
    result.rayCount = rt->getRayCount();
    m_raysPerSecond = 1000.0f * result.rayCount / result.duration;

    printf("\n");

    return result;
}

void Renderer::getTextureParameters(const RaycastResult& hit,
                                    Vec3f& diffuse,
                                    Vec3f& n,
                                    Vec3f& specular) {
    MeshBase::Material* mat = hit.tri->m_material;
    // YOUR CODE HERE (R3):
    // using the barycentric coordinates of the intersection (hit.u, hit.v) and
    // the vertex texture coordinates hit.tri->m_vertices[i].t of the
    // intersected triangle, compute the uv coordinate of the intersection
    // point.
    Vec2f uv{(1.0f - (hit.u + hit.v)) * hit.tri->m_vertices[0].t +
             hit.u * hit.tri->m_vertices[1].t +
             hit.v * hit.tri->m_vertices[2].t};

    Texture& diffuseTex =
        mat->textures
            [MeshBase::TextureType_Diffuse];  // note: you can fetch other kinds
                                              // of textures like this too. By
                                              // default specular maps,
                                              // displacement maps and alpha
                                              // stencils are loaded too if the
                                              // .mtl file specifies them.
    if (diffuseTex.exists())  // check whether material uses a diffuse texture
    {
        const Image& img = *diffuseTex.getImage();
        // fetch diffuse color from texture
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());

        // YOUR CODE HERE (R3): uncomment the line below once you have
        // implemented getTexelCoords.
        diffuse = img.getVec4f(texelCoords).getXYZ();
    }
    Texture& normalTex = mat->textures[MeshBase::TextureType_Normal];
    if (normalTex.exists() &&
        m_normalMapped)  // check whether material uses a normal map
    {
        const Image& img = *normalTex.getImage();
        // EXTRA: do tangent space normal mapping
        // first, get texel coordinates as above, for the rest, see handout
    }

    // EXTRA: read a value from the specular texture into specular_mult.
    Texture& specularTex = mat->textures[MeshBase::TextureType_Specular];
    if (specularTex.exists()) {
        const Image& img = *specularTex.getImage();
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());
        specular = img.getVec4f(texelCoords).getXYZ();
    }
}

void print(const Vec3f& vec) {
    std::cout << vec[0] << " " << vec[1] << " " << vec[2] << "\n";
}

Vec4f Renderer::computeShadingHeadlight(const RaycastResult& hit,
                                        const CameraControls& cameraCtrl) {
    // get diffuse color
    MeshBase::Material* mat = hit.tri->m_material;
    Vec3f diffuse = mat->diffuse.getXYZ();
    Vec3f n(hit.tri->normal());
    Vec3f specular =
        mat->specular;  // specular color. Not used in requirements, but you can
                        // use this in extras if you wish.

    if (m_useTextures) getTextureParameters(hit, diffuse, n, specular);

    Vec3f viewDir = (hit.point - cameraCtrl.getPosition()).normalized();
    // dot with view ray direction <=> "headlight shading"
    float d = fabs(dot(n, viewDir));

    // https://www.rastertek.com/dx10tut21.html
    // https://learnopengl.com/Lighting/Lighting-maps
    // Vec3f lightDir = (hit.point - m_pointLightPos).normalized();
    Vec3f lightDir = -viewDir;

    float lightIntensity = fabs(dot(n, lightDir));
    // Calculate the reflection vector based on the light intensity, normal
    // vector, and light direction.
    Vec3f reflectionDir = normalize(2 * lightIntensity * n - lightDir);
    float specularCoef =
        pow(fabs(dot(reflectionDir, viewDir)), mat->glossiness);
    specular *= specularCoef;

    // assign gray value (d,d,d). Implicit initialization
    Vec3f shade = d;

    return Vec4f(shade * diffuse + specular * specularCoef, 1.0f);
}

Vec3f getRandomPointHalfSphere(Random& rnd) {
    float x, y,
        sumSquares = 2.0f;  // Initialize with 2.0f so we enter the while loop
    while (sumSquares > 1) {
        x = rnd.getF32(-1, 1);
        y = rnd.getF32(-1, 1);
        sumSquares = FW::pow(x, 2) + FW::pow(y, 2);
    }
    return Vec3f(x, y, FW::sqrt(1 - sumSquares));
}

Vec4f Renderer::computeShadingAmbientOcclusion(RayTracer* rt,
                                               const RaycastResult& hit,
                                               const CameraControls& cameraCtrl,
                                               Random& rnd) {
    // YOUR CODE HERE (R4)
    float eps = 1e-4;
    Vec3f n(hit.tri->normal());
    //  Nudge the hit point very slightly (say, 0.001 units) towards the camera
    Vec3f origin =
        hit.point - eps * FW::normalize(hit.dir);  // cameraCtrl.getPosition();
    auto basis = formBasis(n);
    float numHits{0.0f};
    for (size_t i = 0; i < m_aoNumRays; i++) {
        auto randDirection = basis * getRandomPointHalfSphere(rnd);
        if (rt->raycast(origin, m_aoRayLength * randDirection).tri != nullptr)
            numHits++;
    }

    return Vec4f(Vec3f(1 - numHits / m_aoNumRays), 1.0f);
}

Vec4f Renderer::computeShadingWhitted(RayTracer* rt,
                                      const RaycastResult& hit,
                                      const CameraControls& cameraCtrl,
                                      Random& rnd,
                                      int num_bounces) {
    // EXTRA: implement a whitted integrator
    return Vec4f(.0f);
}

}  // namespace FW
