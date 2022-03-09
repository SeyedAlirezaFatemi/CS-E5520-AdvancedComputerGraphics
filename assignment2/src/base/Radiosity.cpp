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

// --------------------------------------------------------------------------
void Radiosity::vertexTaskFunc(MulticoreLauncher::Task& task) {
    RadiosityContext& ctx = *(RadiosityContext*)task.data;

    if (ctx.m_bForceExit) return;

    // which vertex are we to compute?
    int v = task.idx;

    // fetch vertex and its normal
    Vec3f n = ctx.m_scene->vertex(v).n.normalized();
    Vec3f o = ctx.m_scene->vertex(v).p + 0.01f * n;

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

    // This is the dummy implementation you should remove.
    ctx.m_vecResult[v] = n * 0.5 + 0.5;
    Sleep(1);
    return;
}
// --------------------------------------------------------------------------

void Radiosity::startRadiosityProcess(MeshWithColors* scene,
                                      AreaLight* light,
                                      RayTracer* rt,
                                      int numBounces,
                                      int numDirectRays,
                                      int numHemisphereRays) {
    // put stuff the asyncronous processor needs
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

        // more bounces desired?
        if (m_context.m_currentBounce < m_context.m_numBounces) {
            // move current bounce to prev
            m_context.m_vecPrevBounce = m_context.m_vecCurr;
            ++m_context.m_currentBounce;
            // start new tasks for all vertices
            m_launcher.push(vertexTaskFunc, &m_context, 0, m_context.m_scene->numVertices());
            printf("\nStarting bounce %d\n", m_context.m_currentBounce);
        } else
            printf("\n DONE!\n");
    }
}
// --------------------------------------------------------------------------

}  // namespace FW
