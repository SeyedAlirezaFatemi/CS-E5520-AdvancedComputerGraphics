#include "Radiosity.hpp"

#include "AreaLight.hpp"
#include "RayTracer.hpp"

namespace FW {

// --------------------------------------------------------------------------

Radiosity::~Radiosity() {
    if (isRunning()) {
        m_context.m_bForceExit = true;
        while (m_launcher.getNumTasks() > m_launcher.getNumFinished()) Sleep(1);
        m_launcher.popAll();
    }
}

std::tuple<Vec3f, float> getRandomPointHalfSphere(Random& rnd, int& ind) {
    float x, y,
        sumSquares = 2.0f;  // Initialize with 2.0f so we enter the while loop
    while (sumSquares > 1) {
        // x = rnd.getF32(-1, 1);
        // y = rnd.getF32(-1, 1);

        x = sobol::sample(ind, 0) * 2 - 1;
        y = sobol::sample(ind, 1) * 2 - 1;

        sumSquares = FW::pow(x, 2) + FW::pow(y, 2);
        if (sumSquares > 1) {
            ind++;
        }
    }
    ind++;
    float z = FW::sqrt(1 - sumSquares);
    return std::make_tuple(Vec3f(x, y, z), z / FW_PI);
}

// --------------------------------------------------------------------------
void Radiosity::vertexTaskFunc(MulticoreLauncher::Task& task) {
    RadiosityContext& ctx = *(RadiosityContext*)task.data;

    if (ctx.m_bForceExit) return;

    // which vertex are we to compute?
    int v = task.idx;
    float eps = 1e-4;
    // fetch vertex and its normal
    Vec3f n = ctx.m_scene->vertex(v).n.normalized();
    // Nudge the vertex position very slightly towards the normal
    Vec3f o = ctx.m_scene->vertex(v).p + eps * n;

    // YOUR CODE HERE (R3):
    // This starter code merely puts the color-coded normal into the result.
    // Remove the dummy solution to make your own implementation work.
    //
    // In the first bounce, your task is to compute the direct irradiance
    // falling on this vertex from the area light source.
    // In the subsequent passes, you should compute the irradiance by a
    // hemispherical gathering integral. The commented code below gives you
    // an idea of the loop structure. Note that you also have to account
    // for how diffuse textures modulate the irradiance.

    Random rnd{0};
    Vec3f irr{0.f};
    if (ctx.m_currentBounce == 0) {
        // (R2) - Area Light Source
        // Direct lighting pass => integrate direct illumination by shooting shadow rays to light
        // source
        float pdf;
        Vec3f p{0.f}, to_light{0.f};
        for (size_t i = 0; i < ctx.m_numDirectRays; i++) {
            // Draw sample on light source
            ctx.m_light->sample(pdf, p, i, rnd);
            // Construct vector from current vertex (o) to light sample
            to_light = p - o;
            // Trace shadow ray to see if it's blocked
            auto res = ctx.m_rt->raycast(o, to_light);
            if (res.tri) {
                // Blocked
            } else {
                // Add the appropriate emission, 1/r^2 and clamped cosine terms, accounting for the
                // PDF as well.
                auto cos_theta = FW::max(FW::dot(FW::normalize(to_light), n), 0.f);
                auto cos_theta_l =
                    FW::max(FW::dot(-FW::normalize(to_light), ctx.m_light->getNormal()), 0.f);
                irr += ctx.m_light->getEmission() * cos_theta * cos_theta_l /
                       (to_light.lenSqr() * pdf);
            }
        }
        // Note we are NOT multiplying by PI here;
        // it's implicit in the hemisphere-to-light source area change of variables.
        // The result we are computing is _irradiance_, not radiosity.
        irr /= ctx.m_numDirectRays;
    } else {
        // (R3)
        // OK, time for indirect!
        // Implement hemispherical gathering integral for bounces > 1.
        int ind = 0;
        for (size_t i = 0; i < ctx.m_numHemisphereRays; i++) {
            // Draw a local weighted direction and find out where it hits (if anywhere)
            auto [local_direction, pdf] = getRandomPointHalfSphere(rnd, ind);
            // Get local coordinate system the rays are shot from.
            auto basis = formBasis(n);
            auto direction = basis * local_direction;
            // Make the direction long but not too long to avoid numerical instability in the ray
            // tracer. For our scenes, 100 is a good length. (I know, this special casing sucks.)
            // Shoot ray, see where we hit
            auto res = ctx.m_rt->raycast(o, direction * 100);
            if (res.tri) {
                const auto& triangle = *(res.tri);
                // check for backfaces => don't accumulate if we hit a surface from below!
                if (FW::dot(-FW::normalize(direction), triangle.normal()) < 0) {
                    continue;
                }
                // Fetch barycentric coordinates
                auto u = res.u;
                auto v = res.v;
                auto cos_theta = FW::max(FW::dot(FW::normalize(direction), n), 0.f);
                const auto& indices = triangle.m_data.vertex_indices;
                // Interpolate lighting from previous pass
                Vec3f irradiance = (1.0f - (u + v)) * ctx.m_vecPrevBounce[indices[0]] +
                                   u * ctx.m_vecPrevBounce[indices[1]] +
                                   v * ctx.m_vecPrevBounce[indices[2]];
                const auto mat = triangle.m_material;
                Vec3f& diffuse = mat->diffuse.getXYZ();
                const Texture& diffuseTex = mat->textures[MeshBase::TextureType_Diffuse];
                if (diffuseTex.exists()) {
                    const Vec2f uv{(1.0f - (u + v)) * triangle.m_vertices[0].t +
                                   u * triangle.m_vertices[1].t + v * triangle.m_vertices[2].t};
                    const Image& img = *diffuseTex.getImage();
                    // Fetch diffuse color from texture
                    const Vec2i texelCoords = getTexelCoords(uv, img.getSize());
                    diffuse = img.getVec4f(texelCoords).getXYZ();
                } else {
                    // No texture, use constant albedo from material structure.
                    // Already initialized with this value.
                }
                auto outgoing_radiosity = irradiance * diffuse / FW_PI;
                irr += outgoing_radiosity * cos_theta / pdf;
            }
        }
        irr /= ctx.m_numHemisphereRays;
    }

    if (ctx.m_viewMode == BounceOnly) {
        ctx.m_vecResult[v] = irr;
    } else {
        ctx.m_vecResult[v] += irr;
    }
    ctx.m_vecResultTotal[v] += irr;
    ctx.m_vecCurr[v] = irr;
}
// --------------------------------------------------------------------------

void Radiosity::startRadiosityProcess(MeshWithColors* scene,
                                      AreaLight* light,
                                      RayTracer* rt,
                                      int numBounces,
                                      int numDirectRays,
                                      int numHemisphereRays,
                                      const ViewMode viewMode) {
    // EXTRA
    m_context.m_viewMode = viewMode;
    // Check if something has been computed previously.
    if (viewMode == BounceOnly && m_context.m_scene != nullptr) {
        // Check if we need to compute more
        // - 1 because of the no indirect result
        if (numBounces > m_context.m_bounces.size() - 1) {
            // Let's continue from last bounce.
            m_context.m_numBounces = numBounces;
            m_context.m_bounces.emplace_back();
            // fire away!
            // m_launcher.setNumThreads(m_launcher.getNumCores());	// the solution exe is
            // multithreaded
            m_launcher.setNumThreads(
                1);  // but you have to make sure your code is thread safe before enabling this!
            m_launcher.popAll();
            m_launcher.push(vertexTaskFunc, &m_context, 0, scene->numVertices());
            return;
        } else {
            m_context.m_vecResult = m_context.m_bounces[numBounces];
            return;
        }
    }

    // put stuff the asynchronous processor needs
    m_context.m_scene = scene;
    m_context.m_rt = rt;
    m_context.m_light = light;
    m_context.m_currentBounce = 0;
    m_context.m_numBounces = numBounces;
    m_context.m_numDirectRays = numDirectRays;
    m_context.m_numHemisphereRays = numHemisphereRays;

    // resize all the buffers according to how many vertices we have in the scene
    m_context.m_vecResult.resize(scene->numVertices());
    m_context.m_vecCurr.resize(scene->numVertices());
    m_context.m_vecPrevBounce.resize(scene->numVertices());
    m_context.m_vecResult.assign(scene->numVertices(), Vec3f(0, 0, 0));
    m_context.m_vecCurr.resize(scene->numVertices());

    // EXTRA
    m_context.m_vecResultTotal.resize(scene->numVertices());
    m_context.m_vecResultTotal.assign(scene->numVertices(), Vec3f(0, 0, 0));
    // + 1 because of the no indirect result
    m_context.m_bounces.resize(numBounces + 1);

    m_context.m_vecSphericalC.resize(scene->numVertices());
    m_context.m_vecSphericalX.resize(scene->numVertices());
    m_context.m_vecSphericalY.resize(scene->numVertices());
    m_context.m_vecSphericalZ.resize(scene->numVertices());

    m_context.m_vecSphericalC.assign(scene->numVertices(), Vec3f(0, 0, 0));
    m_context.m_vecSphericalX.assign(scene->numVertices(), Vec3f(0, 0, 0));
    m_context.m_vecSphericalY.assign(scene->numVertices(), Vec3f(0, 0, 0));
    m_context.m_vecSphericalZ.assign(scene->numVertices(), Vec3f(0, 0, 0));

    // fire away!
    // m_launcher.setNumThreads(m_launcher.getNumCores());	// the solution exe is multithreaded
    m_launcher.setNumThreads(
        1);  // but you have to make sure your code is thread safe before enabling this!
    m_launcher.popAll();
    m_launcher.push(vertexTaskFunc, &m_context, 0, scene->numVertices());
}
// --------------------------------------------------------------------------

bool Radiosity::updateMeshColors(std::vector<Vec4f>& spherical1,
                                 std::vector<Vec4f>& spherical2,
                                 std::vector<float>& spherical3,
                                 bool spherical) {
    if (!m_context.m_scene || m_context.m_vecResult.size() == 0) return false;
    // Print progress.
    printf("%.2f%% done     \r",
           100.0f * m_launcher.getNumFinished() / m_context.m_scene->numVertices());

    // Copy irradiance over to the display mesh.
    // Because we want outgoing radiosity in the end, we divide by PI here
    // and let the shader multiply the final diffuse reflectance in. See App::setupShaders() for
    // details.
    for (int i = 0; i < m_context.m_scene->numVertices(); ++i) {
        // Packing data for the spherical harmonic extra.
        // In order to manage with fewer vertex attributes in the shader, the third component is
        // stored as the w components of other actually three-dimensional vectors.
        if (spherical) {
            m_context.m_scene->mutableVertex(i).c = m_context.m_vecSphericalC[i] * (1.0f / FW_PI);
            spherical3[i] = m_context.m_vecSphericalZ[i].x * (1.0f / FW_PI);
            spherical1[i] = Vec4f(m_context.m_vecSphericalX[i], m_context.m_vecSphericalZ[i].y) *
                            (1.0f / FW_PI);
            spherical2[i] = Vec4f(m_context.m_vecSphericalY[i], m_context.m_vecSphericalZ[i].z) *
                            (1.0f / FW_PI);
        } else {
            m_context.m_scene->mutableVertex(i).c = m_context.m_vecResult[i] * (1.0f / FW_PI);
        }
    }
    return true;
}
// --------------------------------------------------------------------------

void Radiosity::checkFinish() {
    // have all the vertices from current bounce finished computing?
    if (m_launcher.getNumTasks() == m_launcher.getNumFinished()) {
        // yes, remove from task list
        m_launcher.popAll();

        // EXTRA
        m_context.m_bounces[m_context.m_currentBounce] = m_context.m_vecCurr;

        // more bounces desired?
        if (m_context.m_currentBounce < m_context.m_numBounces) {
            // move current bounce to prev
            m_context.m_vecPrevBounce = m_context.m_vecCurr;
            ++m_context.m_currentBounce;
            // start new tasks for all vertices
            m_launcher.push(vertexTaskFunc, &m_context, 0, m_context.m_scene->numVertices());
            printf("\nStarting bounce %d\n", m_context.m_currentBounce);
        } else {
            printf("\n DONE!\n");
        }
    }
}
// --------------------------------------------------------------------------

}  // namespace FW
