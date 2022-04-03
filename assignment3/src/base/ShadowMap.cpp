#include "ShadowMap.hpp"

namespace FW {

Mat4f LightSource::getPosToLightClip() const {
    // YOUR CODE HERE (R3)
    // You'll need to construct the matrix that sends world space points to
    // points in the light source's clip space. You'll need to somehow use
    // m_xform, which describes the light source pose, and Mat4f::perspective().
    return Mat4f::perspective(getFOV(), getNear(), getFar()) * m_xform.inverted();
}

void LightSource::renderShadowedScene(GLContext *gl,
                                      MeshWithColors *scene,
                                      const Mat4f &worldToCamera,
                                      const Mat4f &projection,
                                      bool fromLight) {
    // Get or build the shader that renders the scene using the shadow map
    GLContext::Program *prog = gl->getProgram("MeshBase::draw_generic");
    if (!prog) {
        printf("Compiling MeshBase::draw_generic\n");
        prog = new GLContext::Program(
            "#version 120\n" FW_GL_SHADER_SOURCE(
                // VERTEX SHADER BEGINS HERE. This part is executed for every vertex in the scene.
                // The output is the bunch of variables with the 'varying' qualifier; these values
                // will be interpolated across triangles, and passed to the fragment shader (which
                // is below this one). The vertex shader also needs to fill in the variable
                // gl_Position, which determines where on the screen the vertex will be drawn. See
                // below for how that's done.

                // Boilerplate stuff, no need to touch these first lines, but it might be useful to
                // understand what they do because you'll be implementing similar things:
                uniform mat4 posToClip;    // projection * view of _camera_
                uniform mat4 posToCamera;  // view matrix
                uniform mat3 normalToCamera;

                attribute vec3 positionAttrib;  // ! world space coordinates of this vertex
                attribute vec3 normalAttrib;
                attribute vec4 vcolorAttrib;  // (workaround: "colorAttrib" appears to confuse
                                              // certain ATI drivers.)
                attribute vec2 texCoordAttrib;

                centroid varying vec3 positionVarying;
                centroid varying vec3 normalVarying;
                centroid varying vec4 colorVarying;
                varying vec2 texCoordVarying;
                uniform bool renderFromLight;

                // Here starts the part you'll need to be concerned with.
                // ! The uniform variables are the "inputs" to the shader. They are filled in prior
                // to drawing using setUniform() calls (see the end of the renderShadowedScene()
                // function).

                // The transformation to light's clip space.
                // Notice the similarity to the posToClip above.
                uniform mat4 posToLightClip;

                // This 'varying' stuff below will be filled in by your code in the main() function
                // of the vertex shader. OpenGL interpolates each value across the triangles, and
                // passes the values to the fragment (pixel) shader, in correspondingly named
                // variables.

                // ! Your code will need to compute the position of the vertex from the light's
                // point of view. It should be in the clip space of the light, just like gl_Position
                // is in the clip space of the camera.
                varying vec4 posLightClip;

                // Feel free to define whatever else you might need. Use uniforms to bring in stuff
                // from the caller ("C++ side"), and varyings to pass computation results to the
                // fragment shader.

                void main() {
                    // vec3 positionAttrib contains the world space coordinates of this vertex. Make
                    // a homogeneous version:
                    vec4 pos = vec4(positionAttrib, 1.0);
                    // Next we'll fulfill the main duty of the vertex shader by assigning the
                    // gl_Position variable. The value must be the camera clip space coordinates of
                    // the vertex. In the notation of the instructions, this is the computation x~ =
                    // P * M * p, because posToClip contains the readily multiplied projection and
                    // view matrices. So in other words the following line corresponds to something
                    // like gl_Position = cameraProjectionMatrix * cameraViewMatrix * pos;
                    gl_Position = posToClip * pos;

                    // Then do some other stuff that might be useful (texture coordinate
                    // interpolation, etc.)
                    positionVarying = (posToCamera * pos).xyz;
                    normalVarying = normalToCamera * normalAttrib;
                    colorVarying = vcolorAttrib;
                    texCoordVarying = texCoordAttrib;

                    // YOUR CODE HERE (R3): Compute the position of vertex in the light's clip space
                    // and pass it to the fragment shader.
                    posLightClip = posToLightClip * pos;
                    // Debug hack that renders the scene from the light's view, activated by a
                    // toggle button in the UI:
                    if (renderFromLight) {
                        gl_Position = posToLightClip * pos;
                    }
                }
                // VERTEX SHADER ENDS HERE
                ),
            "#version 120\n" FW_GL_SHADER_SOURCE(
                // FRAGMENT SHADER BEGINS HERE. This part is executed for every pixel after the
                // vertex shader is done. The ultimate task of the fragment shader is to assign a
                // value to vec4 gl_FragColor. That value will be written to the screen (or wherever
                // we're drawing, possibly into a texture).
                // #define PI 3.1415926538;
                // Boilerplate, no need to touch:
                uniform bool hasNormals; uniform bool hasDiffuseTexture;
                uniform bool hasAlphaTexture;
                uniform vec4 diffuseUniform;
                uniform vec3 specularUniform;
                uniform float glossiness;
                uniform sampler2D diffuseSampler;
                uniform sampler2D alphaSampler;

                centroid varying vec3 positionVarying;
                centroid varying vec3 normalVarying;
                centroid varying vec4 colorVarying;
                varying vec2 texCoordVarying;

                // Some light-related uniforms that were fed in to the shader from the caller.
                uniform float lightFOVRad;
                uniform vec3 lightPosEye;
                uniform vec3 lightDirEye;
                uniform vec3 lightE;

                // Contains the texture unit that has the shadow map:
                uniform sampler2D shadowSampler;

                // Interpolated varying variables. Note that the names correspond to those in the
                // vertex shader. So whatever you computed and assigned to these in the vertex
                // shader, it's now here.
                varying vec4 posLightClip;

                void main() {
                    // Read the basic surface properties:
                    vec4 diffuseColor = diffuseUniform;
                    vec3 specularColor = specularUniform;

                    if (hasDiffuseTexture) {
                        diffuseColor.rgb = texture2D(diffuseSampler, texCoordVarying).rgb;
                    }

                    diffuseColor *= colorVarying;

                    if (hasAlphaTexture) {
                        diffuseColor.a = texture2D(alphaSampler, texCoordVarying).g;
                    }

                    if (diffuseColor.a <= 0.5) {
                        discard;
                    }
                    // Basic stuff done.

                    // YOUR CODE HERE (R2): Compute diffuse shading and the spot light cone.
                    // The uniforms lightPosEye and lightDirEye contain the light position and
                    // direction in eye coordinates. The varyings positionVarying and normalVarying
                    // contain the point position and normal in eye coordinates. Your task is to
                    // compute the incoming vs surface normal cosine (diffuse shading), incoming vs
                    // light direction cosine (because we're assuming cosine distribution within the
                    // cone), and inverse square distance from the light. Multiply these all
                    // together with lightE (light intensity), and that's the shading.
                    //
                    // Figure out a way to determine if we're outside the spot light cone. If so,
                    // set the color to black. Preferably you should make it fade to black smoothly
                    // towards the edges, because otherwise the image will be full of nasty rough
                    // edges. The cone opening angle is given by the lightFOVRad uniform.
                    //
                    // Be careful with the lightFOVRad -- it contains the angle of full angle of
                    // opening, which is twice the angle of the cone against its axis. Often times
                    // you want the latter.
                    vec3 shading = diffuseColor.rgb;
                    float cone = 1.0;
                    // lightDirEye is the cone axis
                    // lightPosEye lightFOVRad positionVarying normalVarying
                    // min with 10 because the indirect lights can cause some very bright spots if
                    // they land near corners.
                    float inverseSquareDistance =
                        min(1.0 / dot(positionVarying - lightPosEye, positionVarying - lightPosEye),
                            10);
                    vec3 incoming = normalize(positionVarying - lightPosEye);
                    float diffuseSurface = max(-dot(incoming, normalVarying), 0);
                    float diffuseLight = max(dot(incoming, lightDirEye), 0);
                    float cosWithConeAxis = max(dot(incoming, lightDirEye), 0);
                    // 1 / PI = 0.3183098861624923
                    shading *= diffuseLight * diffuseSurface * (0.3183098861624923) *
                               inverseSquareDistance * lightE;
                    cone = max(0,
                               min(1,
                                   4 * (cosWithConeAxis - cos(lightFOVRad / 2)) /
                                       (1 - cos(lightFOVRad / 2))));
                    // YOUR CODE HERE (R3):
                    // Here you need to transform the position in light's clip space into light's
                    // NDC space to get the depth value and the UV coordinates for reading the
                    // stored depth value. Once you have those, you need to compare the NDC depth
                    // value to the value you read from the depth texture to detect whether the
                    // fragment / pixel receives light.
                    //
                    // Be aware, though, that the NDC coordinates will  be in range [-1, 1] whereas
                    // UV coordinates need to be in range [0, 1], and the value fetch from the
                    // GL_DEPTH_COMPONENT depth texture will return a number in range [0, 1]. You
                    // can transform between these spaces by affine transformations, i.e. multiply,
                    // then add.
                    float shadow = 1.0;
                    vec3 ndcLight = (posLightClip / posLightClip.w).xyz;
                    // depth from the shadow map. Mapped to [-1, 1]
                    float depth = texture2D(shadowSampler, (ndcLight.xy + 1) * 0.5).g * 2 - 1;
                    shadow = ndcLight.z <= depth ? 1.0 : 0.0;

                    vec3 result = shading * shadow * cone;

                    // Set the alpha value to 1.0, we're not using it for anything and other values
                    // might or might not cause weird things.
                    float alpha = 1.0;

                    // Finally, assign the color to the pixel.
                    gl_FragColor = vec4(result, alpha);
                }
                // FRAGMENT SHADER ENDS HERE
                ));
        gl->setProgram("MeshBase::draw_generic", prog);
    }
    prog->use();

    // Attach the shadow map to the texture slot 5 and tell the shader about it by setting a uniform
    glActiveTexture(GL_TEXTURE0 + 5);
    glBindTexture(GL_TEXTURE_2D, m_shadowMapTexture);
    gl->setUniform(prog->getUniformLoc("shadowSampler"), 5);

    // Set the input parameters to the shader.

    // Here we set the light source transformation according to your getPosToLightClip()
    // implementation.
    gl->setUniform(prog->getUniformLoc("posToLightClip"), getPosToLightClip());

    // Other light source parameters.
    gl->setUniform(
        prog->getUniformLoc("lightPosEye"),
        (worldToCamera * Vec4f(getPosition(), 1.0f)).getXYZ());  // Transformed to eye coords.
    gl->setUniform(prog->getUniformLoc("lightDirEye"),
                   (worldToCamera * Vec4f(getNormal(), 0.0f)).getXYZ());
    gl->setUniform(prog->getUniformLoc("lightE"), getEmission());
    gl->setUniform(prog->getUniformLoc("lightFOVRad"), getFOVRad());
    gl->setUniform(prog->getUniformLoc("renderFromLight"), fromLight);

    scene->draw(gl, worldToCamera, projection);

    glUseProgram(0);
}

void LightSource::renderShadowMap(FW::GLContext *gl,
                                  MeshWithColors *scene,
                                  ShadowMapContext *sm,
                                  bool debug) {
    // suppress unused warning
    (void)debug;

    // Get or build the shader that outputs depth values
    static const char *progId = "ShadowMap::renderDepthTexture";
    GLContext::Program *prog = gl->getProgram(progId);
    if (!prog) {
        printf("Creating shadow map program\n");

        prog = new GLContext::Program(
            FW_GL_SHADER_SOURCE(
                // Have a look at the comments in the main shader in the renderShadowedScene()
                // function in case you are unfamiliar with GLSL shaders.

                // This is the shader that will render the scene from the light source's point of
                // view, and output the depth value at every pixel. Please note that the fragment
                // shader is empty because only the depth buffer produced by OpenGL will be needed.

                // VERTEX SHADER

                // ! The posToLightClip uniform will be used to transform the world coordinate
                // points of the scene. They should end up in the clip coordinates of the light
                // source "camera". If you look below, you'll see that its value is set based on the
                // function getWorldToLightClip(), which you'll need to implement.
                uniform mat4 posToLightClip; attribute vec3 positionAttrib;

                void main() {
                    // YOUR CODE HERE (R3): Transform the vertex to the light's clip space.
                    gl_Position = posToLightClip * vec4(positionAttrib, 1.0);
                }),
            FW_GL_SHADER_SOURCE(
                // FRAGMENT SHADER
                void main(){
                    // No need to do anything normally.
                }));

        gl->setProgram(progId, prog);
    }
    prog->use();

    // If this light source does not yet have an allocated OpenGL texture, then get one
    if (m_shadowMapTexture == 0) {
        m_shadowMapTexture = sm->allocateDepthTexture();
    }

    // Attach the shadow rendering state (off-screen buffers, render target texture)
    sm->attach(gl, m_shadowMapTexture);

    // Here's a trick: we can cull the front faces instead of the backfaces; this
    // moves the shadow map bias nastiness problems to the dark side of the objects
    // where they might not be visible.
    //
    // You may want to comment these lines during debugging, as the front-culled
    // shadow map may look a bit weird.
    glEnable(GL_CULL_FACE);
    glCullFace(GL_FRONT);

    // Clear the texture to maximum distance (1.0 in NDC coordinates)
    glClearDepth(1.0f);
    glClear(GL_DEPTH_BUFFER_BIT);
    glEnable(GL_DEPTH_TEST);

    // Set the transformation matrix uniform.
    gl->setUniform(prog->getUniformLoc("posToLightClip"), getPosToLightClip());

    renderSceneRaw(gl, scene, prog);

    GLContext::checkErrors();

    // Detach off-screen buffers.
    glBindFramebuffer(GL_FRAMEBUFFER, 0);
    glDrawBuffer(GL_BACK);
}

Vec3f getRandomPointHalfSphere(Random &rnd, float r) {
    float x, y,
        sumSquares = 2.0f;  // Initialize with 2.0f so we enter the while loop
    while (sumSquares > 1) {
        x = rnd.getF32(-1, 1);
        y = rnd.getF32(-1, 1);
        sumSquares = FW::pow(x, 2) + FW::pow(y, 2);
    }
    return Vec3f(x * r, y * r, FW::sqrt(1 - sumSquares * FW::pow(r, 2)));
}

void LightSource::sampleEmittedRays(int num,
                                    std::vector<Vec3f> &origs,
                                    std::vector<Vec3f> &dirs,
                                    std::vector<Vec3f> &E_times_pdf) const {
    Random rand(1234);  // Use this random number generator, so that we'll get the same rays on
                        // every frame. Otherwise it'll flicker.

    // Pre-allocate space in the output vectors (just a slight optimization)
    origs.reserve(num);
    dirs.reserve(num);
    E_times_pdf.reserve(num);

    // YOUR CODE HERE (R4):
    // Fill the three vectors with #num ray origins, directions, and intensities divided by
    // probability density. Note that the lambert cosine of the diffuse area light will cancel out.

    // sin(f/2) = r
    float f = getFOVRad();
    float radius = FW::sin(f / 2.0f);
    auto basis = formBasis(getNormal());
    for (size_t i = 0; i < num; i++) {
        dirs.push_back(basis * Vec3f(getRandomPointHalfSphere(rand, radius)) * getFar());
        E_times_pdf.push_back(getEmission() * radius * radius / static_cast<float>(num));
        origs.push_back(getPosition());
    }
    // See the instructions.
}

/*************************************************************************************
 * Nasty OpenGL stuff begins here, you should not need to touch the remainder of this
 * file for the basic requirements.
 *
 */

bool ShadowMapContext::setup(Vec2i resolution) {
    m_resolution = resolution;

    printf("Setting up ShadowMapContext... ");
    teardown();  // Delete the existing stuff, if any

    // Create a depth buffer for the normal shading pass.
    glGenRenderbuffers(1, &m_depthRenderbuffer);
    glBindRenderbuffer(GL_RENDERBUFFER, m_depthRenderbuffer);
    glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT32, m_resolution.x, m_resolution.y);
    glBindRenderbuffer(GL_RENDERBUFFER, 0);
    GLContext::checkErrors();

    // Create a framebuffer used for all rendering.
    glGenFramebuffers(1, &m_framebuffer);

    GLContext::checkErrors();

    // Don't attach yet.
    printf("done\n");

    return true;
}

GLuint ShadowMapContext::allocateDepthTexture() {
    printf("Allocating depth texture... ");

    GLuint tex;

    glGenTextures(1, &tex);
    glBindTexture(GL_TEXTURE_2D, tex);
    glTexImage2D(GL_TEXTURE_2D,
                 0,
                 GL_DEPTH_COMPONENT32,
                 m_resolution.x,
                 m_resolution.y,
                 0,
                 GL_DEPTH_COMPONENT,
                 GL_UNSIGNED_BYTE,
                 NULL);

    // 'Nearest' is the correct interpolation when storing the non-linear depth values in the NDC
    // space. Linear may sometimes useful, depending on the context, e.g. if linear z is stored
    // instead.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);

    // In real life you might want to clamp to the borders and set the border color to either
    // maximum depth or zero. We use a deprecated OpenGL version for compatibility and this version
    // does not support GL_CLAMP_TO_BORDER.
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

    printf("done, %i\n", tex);
    return tex;
}

void ShadowMapContext::attach(GLContext *gl, GLuint texture) {
    if (m_framebuffer == 0) {
        printf("Error: ShadowMapContext not initialized\n");
    }

    // Only render to the the depth buffer.
    //
    // If you want to store linear z for some reason, such as easier bias tricks, you need to bind
    // e.g. a float texture to GL_COLOR_ATTACHMENT0 in addition to binding the depth buffer.
    glBindFramebuffer(GL_FRAMEBUFFER, m_framebuffer);
    glFramebufferTexture2D(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_TEXTURE_2D, texture, 0);

    // Do not draw to any buffer since no color attachments are bound.
    glDrawBuffers(0, nullptr);

    glViewport(0, 0, m_resolution.x, m_resolution.y);
}

void LightSource::renderSceneRaw(FW::GLContext *gl,
                                 MeshWithColors *scene,
                                 GLContext::Program *prog) {
    // Just draw the triangles without doing anything fancy; anything else should be done on the
    // caller's side

    int posAttrib = scene->findAttrib(MeshBase::AttribType_Position);
    if (posAttrib == -1) return;

    glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, scene->getVBO().getGLBuffer());
    scene->setGLAttrib(gl, posAttrib, prog->getAttribLoc("positionAttrib"));

    for (int i = 0; i < scene->numSubmeshes(); i++) {
        glDrawElements(GL_TRIANGLES,
                       scene->vboIndexSize(i),
                       GL_UNSIGNED_INT,
                       (void *)(UPTR)scene->vboIndexOffset(i));
    }

    gl->resetAttribs();
}

void LightSource::draw(const Mat4f &worldToCamera,
                       const Mat4f &projection,
                       bool show_axis,
                       bool show_frame,
                       bool show_square) {
    glUseProgram(0);
    if (show_axis) {
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf((float *)&projection);
        glMatrixMode(GL_MODELVIEW);
        //	Mat4f S = Mat4f::scale(Vec3f(m_size,1));
        Mat4f M = worldToCamera * m_xform;
        glLoadMatrixf((float *)&M);

        const float orig[3] = {0, 0, 0};

        const float XP[3] = {1, 0, 0}, YP[3] = {0, 1, 0}, ZP[3] = {0, 0, 1};

        glLineWidth(3.0);

        glBegin(GL_LINES);
        glColor3f(1, 0, 0);
        glVertex3fv(orig);
        glVertex3fv(XP);
        glColor3f(0, 1, 0);
        glVertex3fv(orig);
        glVertex3fv(YP);
        glColor3f(0, 0, 1);
        glVertex3fv(orig);
        glVertex3fv(ZP);
        glEnd();
    }

    if (show_frame) {
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf((float *)&projection);
        glMatrixMode(GL_MODELVIEW);
        //	Mat4f S = Mat4f::scale(Vec3f(m_size,1));
        Mat4f M = worldToCamera * m_xform *
                  Mat4f::scale(Vec3f(tan(m_fov * 3.14159f / 360), tan(m_fov * 3.14159f / 360), 1));
        glLoadMatrixf((float *)&M);
        glLineWidth(1.0);

        glBegin(GL_LINES);
        glColor3f(1, 1, 1);
        glVertex3f(-1, -1, -1);
        glVertex3f(1, -1, -1);
        glVertex3f(1, -1, -1);
        glVertex3f(1, 1, -1);
        glVertex3f(1, 1, -1);
        glVertex3f(-1, 1, -1);
        glVertex3f(-1, 1, -1);
        glVertex3f(-1, -1, -1);

        glVertex3f(-1, -1, -1);
        glVertex3f(0, 0, 0);
        glVertex3f(1, -1, -1);
        glVertex3f(0, 0, 0);
        glVertex3f(1, 1, -1);
        glVertex3f(0, 0, 0);
        glVertex3f(-1, 1, -1);
        glVertex3f(0, 0, 0);
        glEnd();
    }

    if (show_square) {
        glMatrixMode(GL_PROJECTION);
        glLoadMatrixf((float *)&projection);
        glMatrixMode(GL_MODELVIEW);
        Mat4f S = Mat4f::scale(Vec3f(m_size, 1));
        Mat4f M = worldToCamera * m_xform * S;
        glLoadMatrixf((float *)&M);
        glLineWidth(1.0);

        glBegin(GL_TRIANGLES);
        glColor3f(1.0f, 1.0f, 1.0f);
        glVertex3f(1, 1, 0);
        glVertex3f(1, -1, 0);
        glVertex3f(-1, -1, 0);
        glVertex3f(1, 1, 0);
        glVertex3f(-1, -1, 0);
        glVertex3f(-1, 1, 0);
        glEnd();
    }
}

}  // namespace FW
