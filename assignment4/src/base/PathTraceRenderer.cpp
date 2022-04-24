#include "PathTraceRenderer.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <string>

#include "AreaLight.hpp"
#include "RayTracer.hpp"

namespace FW {

bool PathTraceRenderer::m_normalMapped = false;
bool PathTraceRenderer::debugVis = false;

void PathTraceRenderer::getTextureParameters(const RaycastResult& hit,
                                             Vec3f& diffuse,
                                             Vec3f& n,
                                             Vec3f& specular) {
    MeshBase::Material* mat = hit.tri->m_material;
    // YOUR CODE HERE (R1)
    // Read value from albedo texture into diffuse.
    // If textured, use the texture; if not, use Material.diffuse.
    // Note: You can probably reuse parts of the radiosity assignment.
    Vec2f uv{(1.0f - (hit.u + hit.v)) * hit.tri->m_vertices[0].t +
             hit.u * hit.tri->m_vertices[1].t + hit.v * hit.tri->m_vertices[2].t};
    Texture& diffuseTex =
        mat->textures[MeshBase::TextureType_Diffuse];  // note: you can fetch other kinds
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
        diffuse = img.getVec4f(texelCoords).getXYZ();
    }
    Texture& normalTex = mat->textures[MeshBase::TextureType_Normal];
    if (normalTex.exists() && m_normalMapped)  // check whether material uses a normal map
    {
        const Image& img = *normalTex.getImage();
        // Do tangent space normal mapping
        // first, get texel coordinates as above, for the rest, see handout
        // http://www.opengl-tutorial.org/intermediate-tutorials/tutorial-13-normal-mapping/
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());
        auto normal = 2.0f * img.getVec4f(texelCoords).getXYZ() - 1.0f;
        const auto& v0 = hit.tri->m_vertices[0].p;
        const auto& v1 = hit.tri->m_vertices[1].p;
        const auto& v2 = hit.tri->m_vertices[2].p;
        const auto& uv0 = hit.tri->m_vertices[0].t;
        const auto& uv1 = hit.tri->m_vertices[1].t;
        const auto& uv2 = hit.tri->m_vertices[2].t;
        // Edges of the triangle : position delta
        auto deltaPos1 = v1 - v0;
        auto deltaPos2 = v2 - v0;
        // UV delta
        auto deltaUV1 = uv1 - uv0;
        auto deltaUV2 = uv2 - uv0;
        float r = 1.0f / (deltaUV1.x * deltaUV2.y - deltaUV1.y * deltaUV2.x);
        auto tangent = normalize((deltaPos1 * deltaUV2.y - deltaPos2 * deltaUV1.y) * r);
        auto bitangent = normalize((deltaPos2 * deltaUV1.x - deltaPos1 * deltaUV2.x) * r);
        auto triangleNormal = hit.tri->normal();
        Mat3f tbn;
        tbn.setCol(0, tangent);
        tbn.setCol(1, bitangent);
        tbn.setCol(2, triangleNormal);
        n = tbn * normal;
    } else {
        // Get smooth normals
        // Interpolate the vertex normals to the hit position and normalize it
        n = (1.0f - (hit.u + hit.v)) * hit.tri->m_vertices[0].n + hit.u * hit.tri->m_vertices[1].n +
            hit.v * hit.tri->m_vertices[2].n;
    }
    n = normalize(n);
    // Read a value from the specular texture into specular_mult.
    Texture& specularTex = mat->textures[MeshBase::TextureType_Specular];
    if (specularTex.exists()) {
        const Image& img = *specularTex.getImage();
        Vec2i texelCoords = getTexelCoords(uv, img.getSize());
        specular = img.getVec4f(texelCoords).getXYZ();
    }
}

PathTracerContext::PathTracerContext()
    : m_bForceExit(false),
      m_bResidual(false),
      m_scene(nullptr),
      m_rt(nullptr),
      m_light(nullptr),
      m_pass(0),
      m_bounces(0),
      m_destImage(0),
      m_camera(nullptr) {}

PathTracerContext::~PathTracerContext() {}

PathTraceRenderer::PathTraceRenderer() { m_raysPerSecond = 0.0f; }

PathTraceRenderer::~PathTraceRenderer() { stop(); }

// This function traces a single path and returns the resulting color value that will get rendered
// on the image. Filling in the blanks here is all you need to do this time around.
Vec3f PathTraceRenderer::tracePath(float image_x,
                                   float image_y,
                                   PathTracerContext& ctx,
                                   int samplerBase,
                                   Random& R,
                                   std::vector<PathVisualizationNode>& visualization) {
    static constexpr float RR_TERMINATION_THRESH = .2f;

    const MeshWithColors* scene = ctx.m_scene;
    RayTracer* rt = ctx.m_rt;
    Image* image = ctx.m_image.get();
    const CameraControls& cameraCtrl = *ctx.m_camera;
    AreaLight* light = ctx.m_light;

    // make sure we're on CPU
    // image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) *
                       cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // Simple ray generation code, you can use this if you want to.

    // Generate a ray through the pixel.
    // Randomize coordinates in the pixel.
    float x = ((float)image_x + R.getF32(0.f, 1.f)) / image->getSize().x * 2.0f - 1.0f;
    float y = ((float)image_y + R.getF32(0.f, 1.f)) / image->getSize().y * -2.0f + 1.0f;
    // float x = (float)image_x / image->getSize().x * 2.0f - 1.0f;
    // float y = (float)image_y / image->getSize().y * -2.0f + 1.0f;

    // point on front plane in homogeneous coordinates
    Vec4f P0(x, y, 0.0f, 1.0f);
    // point on back plane in homogeneous coordinates
    Vec4f P1(x, y, 1.0f, 1.0f);

    // apply inverse projection, divide by w to get object-space points
    Vec4f Roh = (invP * P0);
    Vec3f Ro = (Roh * (1.0f / Roh.w)).getXYZ();
    Vec4f Rdh = (invP * P1);
    Vec3f Rd = (Rdh * (1.0f / Rdh.w)).getXYZ();

    // Subtract front plane point from back plane point,
    // yields ray direction.
    // NOTE that it's not normalized; the direction Rd is defined
    // so that the segment to be traced is [Ro, Ro+Rd], i.e.,
    // intersections that come _after_ the point Ro+Rd are to be discarded.
    Rd = Rd - Ro;

    // trace!
    RaycastResult result = rt->raycast(Ro, Rd);
    const RTTriangle* pHit = result.tri;

    // if we hit something, fetch a color and insert into image
    Vec3f Ei{0.f};
    Vec3f throughput{1.f};
    bool rr = false;
    bool rrStarted = false;
    int bounces = ctx.m_bounces;
    if (ctx.m_bounces < 0) {
        rr = true;
        bounces = -bounces;
    }
    if (result.tri != nullptr) {
        // YOUR CODE HERE (R2-R4):
        // Implement path tracing with direct light and shadows, scattering and Russian roulette.
        while (result.tri != nullptr) {
            // Doesn't happen in the scene.
            // Ei += result.tri->m_material->emission;
            MeshBase::Material* mat = result.tri->m_material;
            Vec3f diffuse = mat->diffuse.getXYZ();
            Vec3f n(result.tri->normal());  // Will be smoothed in getTextureParameters
            Vec3f specular = mat->specular;
            getTextureParameters(result, diffuse, n, specular);
            // Gamma correction
            diffuse.x = pow(diffuse.x, 2.2f);
            diffuse.y = pow(diffuse.y, 2.2f);
            diffuse.z = pow(diffuse.z, 2.2f);
            // Nudge the hit point very slightly towards the normal
            Vec3f hitPoint = result.point + 1e-5f * n;
            // Shadow ray
            float lightPDF;
            Vec3f lightPoint;
            light->sample(lightPDF, lightPoint, 0, R);
            auto toLight = lightPoint - hitPoint;
            RaycastResult shadowResult = rt->raycast(hitPoint, toLight);
            if (shadowResult.tri == nullptr) {
                // Light visible
                auto cosThetaL = max(light->getNormal().dot(normalize(-toLight)), 0.f);
                auto cosTheta = max(n.dot(normalize(toLight)), 0.f);
                Ei += throughput * diffuse * light->getEmission() * cosTheta * cosThetaL /
                      (lightPDF * lenSqr(toLight) * FW_PI);
            }

            if (debugVis) {
                // Example code for using the visualization system. You can expand this to include
                // further bounces, shadow rays, and whatever other useful information you can think
                // of.
                PathVisualizationNode node;
                node.lines.push_back(PathVisualizationLine(
                    result.orig, result.point));  // Draws a line between two points
                node.lines.push_back(PathVisualizationLine(
                    result.point,
                    result.point + result.tri->normal() * .1f,
                    Vec3f(1, 0, 0)));  // You can give lines a color as optional parameter.
                node.labels.push_back(PathVisualizationLabel(
                    "diffuse: " + std::to_string(Ei.x) + ", " + std::to_string(Ei.y) + ", " +
                        std::to_string(Ei.z),
                    result.point));  // You can also render text labels with world-space locations.

                visualization.push_back(node);
            }

            if (bounces == 0 && !rr) {
                // We're done
                break;
            }
            // Continue the path
            auto [local_direction, pdf] = getRandomPointHalfSphere(R);
            auto basis = formBasis(n);
            auto direction = normalize(basis * local_direction);
            result = rt->raycast(hitPoint, direction * 100);
            throughput *= diffuse * direction.dot(n) / (FW_PI * pdf);
            if (bounces == 0 && !rrStarted) {
                rrStarted = true;
                throughput *= (1.0f / RR_TERMINATION_THRESH);
            }
            if (rrStarted) {
                if (R.getF32(0.f, 1.f) < RR_TERMINATION_THRESH) {
                    break;
                }
            }
            bounces--;
        }
    }
    return Ei;
}

std::tuple<Vec3f, float> getRandomPointHalfSphere(Random& rnd) {
    float x, y,
        sumSquares = 2.0f;  // Initialize with 2.0f so we enter the while loop
    while (sumSquares > 1) {
        x = rnd.getF32(-1, 1);
        y = rnd.getF32(-1, 1);
        sumSquares = FW::pow(x, 2) + FW::pow(y, 2);
    }
    float z = FW::sqrt(1 - sumSquares);
    return std::make_tuple(Vec3f(x, y, z), z / FW_PI);
}

// This function is responsible for asynchronously generating paths for a given block.
void PathTraceRenderer::pathTraceBlock(MulticoreLauncher::Task& t) {
    PathTracerContext& ctx = *(PathTracerContext*)t.data;

    const MeshWithColors* scene = ctx.m_scene;
    RayTracer* rt = ctx.m_rt;
    Image* image = ctx.m_image.get();
    const CameraControls& cameraCtrl = *ctx.m_camera;
    AreaLight* light = ctx.m_light;

    // make sure we're on CPU
    image->getMutablePtr();

    // get camera orientation and projection
    Mat4f worldToCamera = cameraCtrl.getWorldToCamera();
    Mat4f projection = Mat4f::fitToView(Vec2f(-1, -1), Vec2f(2, 2), image->getSize()) *
                       cameraCtrl.getCameraToClip();

    // inverse projection from clip space to world space
    Mat4f invP = (projection * worldToCamera).inverted();

    // get the block which we are rendering
    PathTracerBlock& block = ctx.m_blocks[t.idx];

    // Not used but must be passed to tracePath
    std::vector<PathVisualizationNode> dummyVisualization;

    static std::atomic<uint32_t> seed = 0;
    uint32_t current_seed = seed.fetch_add(1);
    Random R(t.idx +
             current_seed);  // this is bogus, just to make the random numbers change each iteration

    for (int i = 0; i < block.m_width * block.m_height; ++i) {
        if (ctx.m_bForceExit) {
            return;
        }

        // Use if you want.
        int pixel_x = block.m_x + (i % block.m_width);
        int pixel_y = block.m_y + (i / block.m_width);

        Vec3f Ei = tracePath(pixel_x, pixel_y, ctx, 0, R, dummyVisualization);

        // Put pixel.
        Vec4f prev = image->getVec4f(Vec2i(pixel_x, pixel_y));
        // Update previous value
        prev += Vec4f(Ei, 1.0f);
        image->setVec4f(Vec2i(pixel_x, pixel_y), prev);
    }
}

void PathTraceRenderer::startPathTracingProcess(const MeshWithColors* scene,
                                                AreaLight* light,
                                                RayTracer* rt,
                                                Image* dest,
                                                int bounces,
                                                const CameraControls& camera) {
    FW_ASSERT(!m_context.m_bForceExit);

    m_context.m_bForceExit = false;
    m_context.m_bResidual = false;
    m_context.m_camera = &camera;
    m_context.m_rt = rt;
    m_context.m_scene = scene;
    m_context.m_light = light;
    m_context.m_pass = 0;
    m_context.m_bounces = bounces;
    m_context.m_image.reset(new Image(dest->getSize(), ImageFormat::RGBA_Vec4f));

    m_context.m_destImage = dest;
    m_context.m_image->clear();

    // Add rendering blocks.
    m_context.m_blocks.clear();
    {
        int block_size = 32;
        int image_width = dest->getSize().x;
        int image_height = dest->getSize().y;
        int block_count_x = (image_width + block_size - 1) / block_size;
        int block_count_y = (image_height + block_size - 1) / block_size;

        for (int y = 0; y < block_count_y; ++y) {
            int block_start_y = y * block_size;
            int block_end_y = FW::min(block_start_y + block_size, image_height);
            int block_height = block_end_y - block_start_y;

            for (int x = 0; x < block_count_x; ++x) {
                int block_start_x = x * block_size;
                int block_end_x = FW::min(block_start_x + block_size, image_width);
                int block_width = block_end_x - block_start_x;

                PathTracerBlock block;
                block.m_x = block_size * x;
                block.m_y = block_size * y;
                block.m_width = block_width;
                block.m_height = block_height;

                m_context.m_blocks.push_back(block);
            }
        }
    }

    dest->clear();

    // Fire away!

    // If you change this, change the one in checkFinish too.
    m_launcher.setNumThreads(m_launcher.getNumCores());
    // m_launcher.setNumThreads(1);

    m_launcher.popAll();
    m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size());
}

void PathTraceRenderer::updatePicture(Image* dest) {
    FW_ASSERT(m_context.m_image != 0);
    FW_ASSERT(m_context.m_image->getSize() == dest->getSize());

    for (int i = 0; i < dest->getSize().y; ++i) {
        for (int j = 0; j < dest->getSize().x; ++j) {
            Vec4f D = m_context.m_image->getVec4f(Vec2i(j, i));
            if (D.w != 0.0f) D = D * (1.0f / D.w);

            // Gamma correction.
            Vec4f color = Vec4f(FW::pow(D.x, 1.0f / 2.2f),
                                FW::pow(D.y, 1.0f / 2.2f),
                                FW::pow(D.z, 1.0f / 2.2f),
                                D.w);

            dest->setVec4f(Vec2i(j, i), color);
        }
    }
}

void PathTraceRenderer::checkFinish() {
    // have all the vertices from current bounce finished computing?
    if (m_launcher.getNumTasks() == m_launcher.getNumFinished()) {
        // yes, remove from task list
        m_launcher.popAll();

        ++m_context.m_pass;

        // you may want to uncomment this to write out a sequence of PNG images
        // after the completion of each full round through the image.
        // String fn = sprintf( "pt-%03dppp.png", m_context.m_pass );
        // File outfile( fn, File::Create );
        // exportLodePngImage( outfile, m_context.m_destImage );

        if (!m_context.m_bForceExit) {
            // keep going

            // If you change this, change the one in startPathTracingProcess too.
            m_launcher.setNumThreads(m_launcher.getNumCores());
            // m_launcher.setNumThreads(1);

            m_launcher.popAll();
            m_launcher.push(pathTraceBlock, &m_context, 0, (int)m_context.m_blocks.size());
            //::printf( "Next pass!" );
        } else
            ::printf("Stopped.");
    }
}

void PathTraceRenderer::stop() {
    m_context.m_bForceExit = true;

    if (isRunning()) {
        m_context.m_bForceExit = true;
        while (m_launcher.getNumTasks() > m_launcher.getNumFinished()) {
            Sleep(1);
        }
        m_launcher.popAll();
    }

    m_context.m_bForceExit = false;
}

}  // namespace FW
