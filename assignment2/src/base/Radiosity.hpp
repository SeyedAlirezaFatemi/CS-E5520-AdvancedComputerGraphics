#pragma once

#include <3d/Mesh.hpp>
#include <base/Math.hpp>
#include <base/MulticoreLauncher.hpp>
#include <base/Random.hpp>
#include <vector>

#include "sobol.hpp"
#include "util.hpp"

namespace FW {

class AreaLight;
class RayTracer;

float sphericalHarmonicFactor(int order, int index);

std::tuple<Vec3f, float> getRandomPointHalfSphere(Random& rnd);

//------------------------------------------------------------------------

typedef Mesh<VertexPNTC> MeshWithColors;

//------------------------------------------------------------------------

class Radiosity {
   public:
    Radiosity() {}
    ~Radiosity();

    // we'll compute radiosity asynchronously, meaning we'll be able to
    // fly around in the scene watching the process complete.
    void startRadiosityProcess(MeshWithColors* scene,
                               AreaLight* light,
                               RayTracer* rt,
                               int numBounces,
                               int numDirectRays,
                               int numHemisphereRays,
                               const ViewMode viewMode);

    // are we still processing?
    bool isRunning() const { return m_launcher.getNumTasks() > 0; }
    // sees if we need to switch bounces, etc.
    void checkFinish();

    // copy the current solution to the mesh colors for display. returns true if successful.
    bool updateMeshColors(std::vector<Vec4f>& spherical1,
                          std::vector<Vec4f>& spherical2,
                          std::vector<float>& spherical3,
                          bool spherical);

   protected:
    MulticoreLauncher m_launcher;

    // This structure holds all the variables needed for computing the illumination
    // in multithreaded fashion. See Radiosity::vertexTaskFunc()
    struct RadiosityContext {
        RadiosityContext()
            : m_scene(nullptr),
              m_numBounces(1),
              m_numDirectRays(64),
              m_numHemisphereRays(256),
              m_currentBounce(0),
              m_bForceExit(false) {}

        MeshWithColors* m_scene;
        AreaLight* m_light;
        RayTracer* m_rt;

        int m_numBounces;
        int m_numDirectRays;
        int m_numHemisphereRays;
        int m_currentBounce = -1;

        bool m_bForceExit;

        // these are vectors with one value per vertex
        std::vector<Vec3f> m_vecCurr;  // this one holds the results for the bounce currently being
                                       // computed. Zero in the beginning.
        std::vector<Vec3f>
            m_vecPrevBounce;  // Once a bounce finishes, the results are copied here to be used as
                              // the input illumination to the next bounce.
        std::vector<Vec3f>
            m_vecResult;  // This vector should hold the result to be displayed (used for display).

        std::vector<Vec3f> m_vecResultTotal;  // This vector should hold a sum of all bounces (NOT
                                              // used for display).

        // EXTRA
        std::vector<std::vector<Vec3f>> m_bounces;

        std::vector<Vec3f> m_vecSphericalC, m_vecSphericalX, m_vecSphericalY,
            m_vecSphericalZ;  // For the spherical harmonics extra: these hold the sums for
                              // spherical harmonic coefficients, similarly to m_vecResult.

        ViewMode m_viewMode{FinalResult};
    };

    static void vertexTaskFunc(MulticoreLauncher::Task&);

    RadiosityContext m_context;
};

}  // namespace FW
