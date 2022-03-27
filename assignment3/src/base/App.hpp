#pragma once

#include <memory>
#include <vector>

#include "3d/CameraControls.hpp"
#include "InstantRadiosity.hpp"
#include "RayTracer.hpp"
#include "ShadowMap.hpp"
#include "gpu/Buffer.hpp"
#include "gui/CommonControls.hpp"
#include "gui/Image.hpp"
#include "gui/Window.hpp"

namespace FW {

//------------------------------------------------------------------------

class App : public Window::Listener, public CommonControls::StateObject {
   private:
    enum Action {
        Action_None,

        Action_LoadMesh,
        Action_ReloadMesh,
        Action_SaveMesh,
        Action_LoadBVH,

        Action_ResetCamera,
        Action_EncodeCameraSignature,
        Action_DecodeCameraSignature,

        Action_NormalizeScale,
        Action_FlipXY,
        Action_FlipYZ,
        Action_FlipZ,

        Action_NormalizeNormals,
        Action_FlipNormals,
        Action_RecomputeNormals,

        Action_FlipTriangles,

        Action_CleanMesh,
        Action_CollapseVertices,
        Action_DupVertsPerSubmesh,
        Action_FixMaterialColors,
        Action_DownscaleTextures,
        Action_ChopBehindNear,

        Action_TracePrimaryRays,
        Action_PlaceLightSourceAtCamera
    };

    enum CullMode {
        CullMode_None = 0,
        CullMode_CW,
        CullMode_CCW,
    };

    struct RayVertex {
        Vec3f pos;
        U32 color;
    };

    enum bvh_build_method { None, SAH };
    enum SamplingType { AO_sampling, AA_sampling };
    // this structure holds the necessary arguments when rendering using command line parameters
    struct {
        bool batch_render;
        SplitMode splitMode;  // the BVH builder to use
        int spp;              // samples per pixel to use
        SamplingType
            sample_type;      // AO or AA sampling; AO includes one extra sample for the primary ray
        bool output_images;   // might be useful to compare images with the example
        bool use_textures;    // whether or not textures are used
        bool use_arealights;  // whether or not area light sampling is used
        bool enable_reflections;  // whether to compute reflections in whitted integrator
        float ao_length;
    } m_settings;

    struct {
        std::string state_name;  // filenames of the state and scene files
        std::string scene_name;
        int rayCount;
        int build_time, trace_time;

    } m_results;

   public:
    App(std::vector<std::string>& cmd_args);
    virtual ~App(void);

    virtual bool handleEvent(const Window::Event& ev);
    virtual void readState(StateDump& d);
    virtual void writeState(StateDump& d) const;

   private:
    void process_args(std::vector<std::string>& args);

    void waitKey(void);
    void renderFrame(GLContext* gl);
    void loadMesh(const String& fileName);
    void saveMesh(const String& fileName);
    void loadRayDump(const String& fileName);

    static void downscaleTextures(MeshBase* mesh);
    static void chopBehindPlane(MeshBase* mesh, const Vec4f& pleq);

    static bool fileExists(const String& fileName);

    //
    void constructTracer(void);

    void blitRttToScreen(GLContext* gl);

    void setupShaders(GLContext* gl);
    void setupRenderToTexture(const Vec2i& resolution);
    void deleteRenderToTexture();
    void blitShadowMapToScreen(GLContext* gl, const LightSource& ls);

   private:
    App(const App&);             // forbidden
    App& operator=(const App&);  // forbidden

   private:
    Window m_window;
    CommonControls m_commonCtrl;
    CameraControls m_cameraCtrl;

    Action m_action;
    String m_meshFileName;
    CullMode m_cullMode;
    Timer m_timer;

    std::unique_ptr<RayTracer> m_rt;
    std::vector<Vec3f> m_rtVertexPositions;  // kept only for MD5 checksums
    std::vector<RTTriangle> m_rtTriangles;

    std::unique_ptr<MeshWithColors> m_mesh;
    std::unique_ptr<LightSource> m_lightSource;
    int m_numHemisphereRays;
    float m_lightSize;
    float m_toneMapWhite;
    float m_toneMapBoost;
    Timer m_updateClock;

    ShadowMapContext m_smcontext;
    InstantRadiosity m_instantRadiosity;
    float m_shadowMapVisMultiplier;
    float m_lightFOV;
    float m_indirectFOV;
    float m_lightIntensity;
    bool m_visualizeLight;
    bool m_visualizeIndirect;
    bool m_renderFromLight;
    int m_num_indirect;
    int m_smResolutionLevel;
    int m_smResolutionLevelPrev;

    GLuint m_rttFBO;
    GLuint m_rttDepth;
    GLuint m_rttTex;
};

//------------------------------------------------------------------------
}  // namespace FW
