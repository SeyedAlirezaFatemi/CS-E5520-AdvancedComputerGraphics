#pragma once

#include <base/Main.hpp>
#include <io/StateDump.hpp>

#include <base/Math.hpp>
#include <base/Random.hpp>
#include <3d/Mesh.hpp>

#include <vector>

#include "RayTracer.hpp"
#include "ShadowMap.hpp"


namespace FW
{


class RayTracer;
class GLContext;


//------------------------------------------------------------------------


typedef Mesh<VertexPNTC>	MeshWithColors;


//------------------------------------------------------------------------


class InstantRadiosity
{
public:
    InstantRadiosity() : 
      m_indirectFOV(150) // Use 150 degree cone by default
    {};
    ~InstantRadiosity() {};

    void setup(GLContext* gl, Vec2i resolution);

    void draw( const Mat4f& worldToCamera, const Mat4f& projection  );
    void castIndirect(RayTracer *rt, MeshWithColors *scene, const LightSource& ls, int num);
    void renderShadowMaps(MeshWithColors *scene);
    GLContext::Program* getShader();

    void setFOV(float fov) { m_indirectFOV = fov; }

    int getNumLights() const { return (int)m_indirectLights.size(); }
    LightSource& getLight(int i) { return m_indirectLights[i]; };

protected:
    GLContext *m_gl;
    ShadowMapContext m_smContext;
    float m_indirectFOV;
    std::vector<LightSource> m_indirectLights;
};


} // namepace FW
