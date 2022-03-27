#define _CRT_SECURE_NO_WARNINGS

#include "App.hpp"
#include "base/Main.hpp"
#include "gpu/GLContext.hpp"
#include "3d/Mesh.hpp"
#include "io/File.hpp"
#include "io/StateDump.hpp"
#include "base/Random.hpp"

#include "RayTracer.hpp"
#include "rtlib.hpp"

#include <stdio.h>
#include <conio.h>
#include <map>
#include <algorithm>
#include <chrono>

#include <iostream>
#include <fstream>
#include <string>

using namespace FW;


//------------------------------------------------------------------------

bool fileExists(std::string fileName)
{
	return std::ifstream(fileName).good();
}

App::App(std::vector<std::string>& cmd_args)
	: m_commonCtrl(CommonControls::Feature_Default & ~CommonControls::Feature_RepaintOnF5),
	m_cameraCtrl(&m_commonCtrl, CameraControls::Feature_Default | CameraControls::Feature_StereoControls),
	m_action(Action_None),
	m_cullMode(CullMode_None),
	m_numHemisphereRays (256),
	m_lightSize         (0.25f),
	m_toneMapWhite      (1.0f),
	m_toneMapBoost      (1.0f),
	m_rttFBO            (0),
	m_rttDepth          (0),
	m_rttTex            (0)
{

	m_commonCtrl.showFPS(true);
	m_commonCtrl.addStateObject(this);
	m_cameraCtrl.setKeepAligned(true);

	m_commonCtrl.addButton((S32*)&m_action, Action_LoadMesh, FW_KEY_M, "Load mesh or state... (M)");
	m_commonCtrl.addButton((S32*)&m_action, Action_ReloadMesh, FW_KEY_F5, "Reload mesh (F5)");
	m_commonCtrl.addButton((S32*)&m_action, Action_SaveMesh, FW_KEY_O, "Save mesh... (O)");
	m_commonCtrl.addButton((S32*)&m_action, Action_LoadBVH, FW_KEY_NONE, "Load BVH from file...");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_ResetCamera, FW_KEY_NONE, "Reset camera");
	m_commonCtrl.addButton((S32*)&m_action, Action_EncodeCameraSignature, FW_KEY_NONE, "Encode camera signature");
	m_commonCtrl.addButton((S32*)&m_action, Action_DecodeCameraSignature, FW_KEY_NONE, "Decode camera signature...");
	m_window.addListener(&m_cameraCtrl);
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_NormalizeScale, FW_KEY_NONE, "Normalize scale");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipXY,                  FW_KEY_NONE,    "Flip X/Y");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipYZ,                  FW_KEY_NONE,    "Flip Y/Z");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FlipZ,                   FW_KEY_NONE,    "Flip Z");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_NormalizeNormals, FW_KEY_NONE, "Normalize normals");
	m_commonCtrl.addButton((S32*)&m_action, Action_FlipNormals, FW_KEY_NONE, "Flip normals");
	m_commonCtrl.addButton((S32*)&m_action, Action_RecomputeNormals, FW_KEY_NONE, "Recompute normals");
	m_commonCtrl.addSeparator();

	m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_None, FW_KEY_NONE, "Disable backface culling");
	m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_CW, FW_KEY_NONE, "Cull clockwise faces");
	m_commonCtrl.addToggle((S32*)&m_cullMode, CullMode_CCW, FW_KEY_NONE, "Cull counter-clockwise faces");
	m_commonCtrl.addButton((S32*)&m_action, Action_FlipTriangles, FW_KEY_NONE, "Flip triangles");
	m_commonCtrl.addSeparator();

	//    m_commonCtrl.addButton((S32*)&m_action, Action_CleanMesh,               FW_KEY_NONE,    "Remove unused materials, denegerate triangles, and unreferenced vertices");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_CollapseVertices,        FW_KEY_NONE,    "Collapse duplicate vertices");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_DupVertsPerSubmesh,      FW_KEY_NONE,    "Duplicate vertices shared between multiple materials");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_FixMaterialColors,       FW_KEY_NONE,    "Override material colors with average over texels");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_DownscaleTextures,       FW_KEY_NONE,    "Downscale textures by 2x");
	//    m_commonCtrl.addButton((S32*)&m_action, Action_ChopBehindNear,          FW_KEY_NONE,    "Chop triangles behind near plane");
	//    m_commonCtrl.addSeparator();

	m_commonCtrl.addButton((S32*)&m_action, Action_PlaceLightSourceAtCamera,FW_KEY_SPACE,   "Place light at camera (SPACE)");

	m_commonCtrl.addToggle(&m_renderFromLight, FW_KEY_NONE, "Render from light source view" );
	m_commonCtrl.addToggle(&m_visualizeLight, FW_KEY_NONE, "Visualize main light source" );
	m_commonCtrl.addToggle(&m_visualizeIndirect, FW_KEY_NONE, "Visualize indirect light sources" );
	m_commonCtrl.beginSliderStack();
	m_commonCtrl.addSlider(&m_smResolutionLevel, 1, 11, false, FW_KEY_NONE, FW_KEY_NONE, "Shadow map resolution= 2^%d");
	m_commonCtrl.addSlider(&m_num_indirect, 0, 256, false, FW_KEY_NONE, FW_KEY_NONE, "Number of indirect lights= %d");
	m_commonCtrl.addSlider(&m_indirectFOV, 1.0f, 180.0f, false, FW_KEY_NONE, FW_KEY_NONE, "Indirect light FOV= %f");
	m_commonCtrl.addSlider(&m_shadowMapVisMultiplier, 0.00001f, 10.0f, true, FW_KEY_NONE, FW_KEY_NONE, "Shadow map visualization intensity= %f");
	m_commonCtrl.endSliderStack();
	m_commonCtrl.beginSliderStack();
	m_commonCtrl.addSlider(&m_lightFOV, 1.0f, 180.0f, false, FW_KEY_NONE, FW_KEY_NONE, "Light source FOV= %f");
	m_commonCtrl.addSlider(&m_lightIntensity, 0.01f, 1000.0f, true, FW_KEY_NONE, FW_KEY_NONE, "Light source intensity= %f");
	m_commonCtrl.endSliderStack();
	m_commonCtrl.beginSliderStack();
	m_commonCtrl.addSlider(&m_toneMapWhite, 0.05f, 10.0f, true, FW_KEY_NONE, FW_KEY_NONE, "Tone mapping parameter 1= %f");
	m_commonCtrl.addSlider(&m_toneMapBoost, 0.005f, 10.0f, true, FW_KEY_NONE, FW_KEY_NONE, "Tone mapping parameter 2= %f");
	m_commonCtrl.endSliderStack();  

	m_window.addListener(this);
	m_window.addListener(&m_commonCtrl);

	m_window.setTitle("Assignment 3");
	m_commonCtrl.setStateFilePrefix("state_assignment3_");

	m_lightSource.reset(new LightSource);
	m_visualizeLight = true;
	m_visualizeIndirect = false;
	m_renderFromLight = false;
	m_instantRadiosity.setup(m_window.getGL(), Vec2i(256, 256));

	m_smcontext.setup(Vec2i(1024, 1024));
	m_lightFOV = 80;
	m_num_indirect = 81;
	m_shadowMapVisMultiplier = 0.00001;
	m_smResolutionLevel = 9;
	m_smResolutionLevelPrev = 9;
	m_indirectFOV = 150;
	m_lightIntensity = 4;

	process_args(cmd_args);



	m_commonCtrl.loadState(m_commonCtrl.getStateFileName(1));
	m_timer.start();
}

// returns the index of the needle in the haystack or -1 if not found
int find_argument(std::string needle, std::vector<std::string> haystack) {

	for (unsigned j = 0; j < haystack.size(); ++j)
		if (!haystack[j].compare(needle))
			return j;
	
	return -1;
}

void App::process_args(std::vector<std::string>& args) {

	// all of the possible cmd arguments and the corresponding enums (enum value is the index of the string in the vector)
	const std::vector<std::string> argument_names = { "-builder", "-spp", "-output_images", "-use_textures", "-bat_render", "-aa", "-ao", "-ao_length" };
	enum argument { arg_not_found = -1, builder = 0, spp = 1, output_images = 2, use_textures = 3, bat_render = 4, AA = 5, AO = 6, AO_length = 7 };

	// similarly a list of the implemented BVH builder types
	const std::vector<std::string> builder_names = { "none", "sah", "object_median", "spatial_median", "linear" };
	enum builder_type { builder_not_found = -1, builder_None = 0, builder_SAH = 1, builder_ObjectMedian = 2, builder_SpatialMedian = 3, builder_Linear = 4 };

	m_settings.batch_render = false;
	m_settings.output_images = false;
	m_settings.use_textures = true;
	m_settings.sample_type = AA_sampling;
	m_settings.ao_length = 1.0f;
	m_settings.spp = 1;
	m_settings.splitMode = SplitMode_Sah;

	for (unsigned i = 0; i < args.size(); ++i) {

		// try to recognize the argument
		argument cmd = argument(find_argument(args[i], argument_names));

		switch (cmd) {

		case bat_render:
			m_settings.batch_render = true;
			break;

		case output_images:
			m_settings.output_images = true;
			break;
		
		case use_textures:
			m_settings.use_textures = true;
			break;
		
		case AO:
			m_settings.sample_type = AO_sampling;
			break;

		case AA:
			m_settings.sample_type = AA_sampling;
			break;

		case spp:
			++i;
			m_settings.spp = std::stoi(args[i]);
			break;

		case AO_length:
			++i;
			m_settings.ao_length = std::stof(args[i]);
			break;

		case builder: {

			++i;
			builder_type type = builder_type(find_argument(args[i], builder_names));

			if (type==builder_not_found) {
				type = builder_SAH;
				std::cout << "BVH builder not recognized, using Surface Area Heuristic" << std::endl;
				break;
			}

			switch (type) {

			case builder_None:
				m_settings.splitMode = SplitMode_None;
				break;
			
			case builder_SAH:
				m_settings.splitMode = SplitMode_Sah;
				break;

			case builder_ObjectMedian:
				m_settings.splitMode = SplitMode_ObjectMedian;
				break;

			case builder_SpatialMedian:
				m_settings.splitMode = SplitMode_SpatialMedian;
				break;

			case builder_Linear:
				m_settings.splitMode = SplitMode_Linear;
				break;
			}

			break;
		}


		default:
			if (args[i][0] == '-')std::cout << "argument \"" << args[i] << "\" not found!" << std::endl;

		}
	}
	if (m_settings.batch_render)
		m_settings.use_textures = false;
}

//------------------------------------------------------------------------

App::~App()
{
	deleteRenderToTexture();
}

//------------------------------------------------------------------------

bool App::handleEvent(const Window::Event& ev)
{
	if (ev.type == Window::EventType_Close)
	{
		m_window.showModalMessage("Exiting...");
		delete this;
		return true;
	}

	if (ev.type == Window::EventType_Resize)
	{
		// Need to reallocate the RTT textures and buffers.
		printf("Resized %i %i\n", m_window.getSize().x, m_window.getSize().y);
		deleteRenderToTexture();
		setupRenderToTexture(m_window.getSize());
	}

	Action action = m_action;
	m_action = Action_None;
	String name;
	Mat4f mat;

	switch (action)
	{
	case Action_None:
		break;


	case Action_LoadMesh:
		name = m_window.showFileLoadDialog("Load mesh or state", getMeshImportFilter()+",dat:State file");
		if (name.getLength())
			if (name.endsWith(".dat"))
				m_commonCtrl.loadState(name);
			else
				loadMesh(name);
		break;

	case Action_ReloadMesh:
		if (m_meshFileName.getLength())
			loadMesh(m_meshFileName);
		break;

	case Action_SaveMesh:
		name = m_window.showFileSaveDialog("Save mesh", getMeshExportFilter());
		if (name.getLength())
			saveMesh(name);
		break;

	case Action_LoadBVH:
		name = m_window.showFileLoadDialog("Load bvh", "hierarchy:BVH");
		m_rt->loadHierarchy(name.getPtr(), m_rtTriangles);
		break;

	case Action_ResetCamera:
		if (m_mesh)
		{
			m_cameraCtrl.initForMesh(m_mesh.get());
			m_commonCtrl.message("Camera reset");
		}
		break;

	case Action_EncodeCameraSignature:
		m_window.setVisible(false);
		printf("\nCamera signature:\n");
		printf("%s\n", m_cameraCtrl.encodeSignature().getPtr());
		waitKey();
		break;

	case Action_DecodeCameraSignature:
	{
		m_window.setVisible(false);
		printf("\nEnter camera signature:\n");

		char buf[1024];
		if (scanf_s("%s", buf, FW_ARRAY_SIZE(buf)))
			m_cameraCtrl.decodeSignature(buf);
		else
			setError("Signature too long!");

		if (!hasError())
			printf("Done.\n\n");
		else
		{
			printf("Error: %s\n", getError().getPtr());
			clearError();
			waitKey();
		}
	}
		break;

	case Action_NormalizeScale:
		if (m_mesh)
		{
			Vec3f lo, hi;
			m_mesh->getBBox(lo, hi);
			m_mesh->xform(Mat4f::scale(Vec3f(2.0f / (hi - lo).max())) * Mat4f::translate((lo + hi) * -0.5f));
		}
		break;

	case Action_FlipXY:
		std::swap(mat.col(0), mat.col(1));
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_FlipYZ:
		std::swap(mat.col(1), mat.col(2));
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_FlipZ:
		mat.col(2) = -mat.col(2);
		if (m_mesh)
		{
			m_mesh->xform(mat);
			m_mesh->flipTriangles();
		}
		break;

	case Action_NormalizeNormals:
		if (m_mesh)
			m_mesh->xformNormals(mat.getXYZ(), true);
		break;

	case Action_FlipNormals:
		mat = -mat;
		if (m_mesh)
			m_mesh->xformNormals(mat.getXYZ(), false);
		break;

	case Action_RecomputeNormals:
		if (m_mesh)
			m_mesh->recomputeNormals();
		break;

	case Action_FlipTriangles:
		if (m_mesh)
			m_mesh->flipTriangles();
		break;

	case Action_CleanMesh:
		if (m_mesh)
			m_mesh->clean();
		break;

	case Action_CollapseVertices:
		if (m_mesh)
			m_mesh->collapseVertices();
		break;

	case Action_DupVertsPerSubmesh:
		if (m_mesh)
			m_mesh->dupVertsPerSubmesh();
		break;

	case Action_FixMaterialColors:
		if (m_mesh)
			m_mesh->fixMaterialColors();
		break;

	case Action_DownscaleTextures:
		if (m_mesh)
			downscaleTextures(m_mesh.get());
		break;

	case Action_ChopBehindNear:
		if (m_mesh)
		{
			Mat4f worldToClip = m_cameraCtrl.getCameraToClip() * m_cameraCtrl.getWorldToCamera();
			Vec4f pleq = worldToClip.getRow(2) + worldToClip.getRow(3);
			chopBehindPlane(m_mesh.get(), pleq);
		}
		break;
	case Action_PlaceLightSourceAtCamera:
		m_lightSource->setOrientation( m_cameraCtrl.getCameraToWorld().getXYZ() );
		m_lightSource->setPosition(m_cameraCtrl.getPosition());
		m_commonCtrl.message("Placed light at camera");
		break;
	default:
		FW_ASSERT(false);
		break;
		}

	if (ev.type == Window::EventType_KeyUp)
	{
	}



	m_window.setVisible(true);

	if (ev.type == Window::EventType_Paint)
		renderFrame(m_window.getGL());
	m_window.repaint();
	return false;
		}

//------------------------------------------------------------------------

void App::readState(StateDump& d)
{
	String meshFileName;

	d.pushOwner("App");
	d.get(meshFileName,     "m_meshFileName");
	d.get((S32&)m_cullMode, "m_cullMode");
	d.get((S32&)m_numHemisphereRays, "m_numHemisphereRays");
	d.get((S32&)m_toneMapWhite, "m_tonemapWhite");
	d.get((S32&)m_toneMapBoost, "m_tonemapBoost");
	d.get((S32&)m_lightIntensity, "m_lightIntensity");
	d.get((S32&)m_lightFOV, "m_lightFOV");
	d.popOwner();

	m_lightSource->readState(d);
	m_lightSize = m_lightSource->getSize().x;	// dirty; doesn't allow for rectangular lights, only square. TODO

	if (m_meshFileName != meshFileName && meshFileName.getLength()) {
		loadMesh(meshFileName);
	}
	}

//------------------------------------------------------------------------

void App::writeState(StateDump& d) const
{
	d.pushOwner("App");
	d.set(m_meshFileName,            "m_meshFileName");
	d.set((S32)m_cullMode,           "m_cullMode");
	d.set((S32&)m_numHemisphereRays, "m_numHemisphereRays");
	d.set((S32&)m_toneMapWhite,      "m_tonemapWhite");
	d.set((S32&)m_toneMapBoost,      "m_tonemapBoost");
	d.set((S32&)m_lightIntensity,    "m_lightIntensity");
	d.set((S32&)m_lightFOV,		     "m_lightFOV");
	d.popOwner();

	m_lightSource->writeState(d);
}

//------------------------------------------------------------------------

void App::waitKey(void)
{
	printf("Press any key to continue . . . ");
	_getch();
	printf("\n\n");
}

//------------------------------------------------------------------------

void App::renderFrame(GLContext* gl)
{
	if(!m_rt) {
		// Cannot render yet.
		return;
	}

	// Set the light source openings according to user's settings
	m_lightSource->setFOV(m_lightFOV);
	m_lightSource->setEmission(Vec3f(m_lightIntensity));
	m_instantRadiosity.setFOV(m_indirectFOV);

	// Dumb hack, won't bother with callbacks or anything now:
	// If the user-set shadow map resolution has changed since last frame, reallocate everything.
	if (m_smResolutionLevelPrev != m_smResolutionLevel)
		m_instantRadiosity.setup(gl, Vec2i(1 << m_smResolutionLevel, 1 << m_smResolutionLevel));	// power-of-two trick
	m_smResolutionLevelPrev = m_smResolutionLevel;

	// Cast the indirect light sources from the main light source using the raytracer
	m_instantRadiosity.castIndirect(m_rt.get(), m_mesh.get(), *m_lightSource, m_num_indirect);

	// Render the shadow maps for all the indirect lights into an off-screen buffer
	m_instantRadiosity.renderShadowMaps(m_mesh.get());
	// Similarly, render the shadow map for the main light
	m_lightSource->renderShadowMap(gl, m_mesh.get(), &m_smcontext);

	// We need to render the screen image into an off-screen floating point buffer, because 
	// we're accumulating very small values and they get rounded if we use a typical 8-bit
	// uint buffer. Let's attach the off-screen drawing buffers:
	glBindFramebuffer(GL_FRAMEBUFFER, m_rttFBO);
	glViewport(0,0,gl->getViewSize().x, gl->getViewSize().y);

	// Setup transformations.
	Mat4f worldToCamera = m_cameraCtrl.getWorldToCamera();
	Mat4f projection = gl->xformFitToView(Vec2f(-1.0f, -1.0f), Vec2f(2.0f, 2.0f)) * m_cameraCtrl.getCameraToClip();

	// Initialize GL state.
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);

	if (m_cullMode == CullMode_None)
		glDisable(GL_CULL_FACE);
	else
	{
		glEnable(GL_CULL_FACE);
		glCullFace(GL_BACK);
		glFrontFace((m_cullMode == CullMode_CW) ? GL_CCW : GL_CW);
	}

	// No mesh => skip.
	if (!m_mesh)
	{
		gl->drawModalMessage("No mesh loaded!");
		glBindFramebuffer(GL_FRAMEBUFFER, 0);
		return;
	}


	// We first render the scene with the main light on; the additive blending is off at this point.
	m_lightSource->renderShadowedScene(gl, m_mesh.get(), worldToCamera, projection, m_renderFromLight);

	// Then loop through all the indirect lights and re-draw the scene with each of them individually.
	// We use an additive blend mode, so that the light gets added on top. The end result will be an
	// image with all the lights on.
	for (int i = 0; i < m_instantRadiosity.getNumLights(); i++)
	{
		// Don't draw if the light is off (e.g. it flew outisde the scene)
		if (!m_instantRadiosity.getLight(i).isEnabled())
			continue;
		glDepthFunc(GL_EQUAL);
		glEnable(GL_BLEND);
		glBlendFunc(GL_ONE, GL_ONE);
		glDepthMask(GL_FALSE);

		m_instantRadiosity.getLight(i).renderShadowedScene(gl, m_mesh.get(), worldToCamera, projection);
	}

	// Reset the default state
	glDisable(GL_BLEND);
	glDepthFunc(GL_LESS);
	glDepthMask(GL_TRUE);

	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glUseProgram(0);

	// Draw the tone-mapped result on the screen
	blitRttToScreen(gl);

	if (m_shadowMapVisMultiplier > 0.000011)
	{
		glDisable(GL_DEPTH_TEST);
		glViewport(0,0,256,256);
		blitShadowMapToScreen(gl, *m_lightSource);
		glViewport(0,0,gl->getViewSize().x, gl->getViewSize().y);
		glEnable(GL_DEPTH_TEST);
	}

	if (!m_renderFromLight)
	{
		// Draw the light source.
		m_lightSource->draw( worldToCamera, projection, false, false, true);

		// Draw some visualizations if desired. This is done here, because otherwise the visualizations
		// will be affected by the tone mapping.
		if (m_visualizeLight)
			m_lightSource->draw( worldToCamera, projection, true, true, false);
		if (m_visualizeIndirect)
			m_instantRadiosity.draw( worldToCamera, projection );
	}

	// Display status line.

	m_commonCtrl.message(sprintf("Triangles = %d, vertices = %d, materials = %d",
		m_mesh->numTriangles(),
		m_mesh->numVertices(),
		m_mesh->numSubmeshes()),
		"meshStats");
}

//------------------------------------------------------------------------




const static F32 posAttrib[] =
{
	-1, -1, 0, 1,
	1, -1, 0, 1,
	-1, 1, 0, 1,
	1, 1, 0, 1
};


const static F32 texAttrib[] =
{
	0, 1,
	1, 1,
	0, 0,
	1, 0
};

void App::blitRttToScreen(GLContext* gl)
{
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glDisable(GL_DEPTH_TEST);


	// Create program.
	const char* progId = "App::blitToScreen";
	GLContext::Program* prog = gl->getProgram(progId);
	if (!prog)
	{
		prog = new GLContext::Program(
			FW_GL_SHADER_SOURCE(
			attribute vec4 posAttrib;
		attribute vec2 texAttrib;
		varying vec2 uv;
		void main()
		{
			gl_Position = posAttrib;
			uv = texAttrib;
		}
		),
			FW_GL_SHADER_SOURCE(
			uniform sampler2D texSampler;
		varying vec2 uv;
		uniform float reinhardLWhite;
		uniform float tonemapBoost;


		void main()
		{
			vec3 color = texture2D(texSampler, vec2(uv.x, 1.0 - uv.y)).rgb;
			color *= tonemapBoost;
			float L = 0.2126 * color.x + 0.7152 * color.y + 0.0722 * color.z;
			color *= (1.0 + L / reinhardLWhite / reinhardLWhite) / (1.0 + L);

			gl_FragColor = vec4(color, 1.0);
		}
		));
		gl->setProgram(progId, prog);
	}

	prog->use();

	gl->setAttrib(prog->getAttribLoc("posAttrib"), 4, GL_FLOAT, 0, posAttrib);
	gl->setAttrib(prog->getAttribLoc("texAttrib"), 2, GL_FLOAT, 0, texAttrib);
	gl->setUniform(prog->getUniformLoc("reinhardLWhite"), m_toneMapWhite);
	gl->setUniform(prog->getUniformLoc("tonemapBoost"), m_toneMapBoost);
	GLint oldTex = 0;
	glGetIntegerv(GL_TEXTURE_BINDING_2D, &oldTex); //get currently bound texture so texture bind state can remain unchanged

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, m_rttTex);
	gl->setUniform(prog->getUniformLoc("texSampler"), 0);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
	glBindTexture(GL_TEXTURE_2D, oldTex);
	GLContext::checkErrors();
}






void App::blitShadowMapToScreen(GLContext* gl, const LightSource& ls)
{
	glDisable(GL_DEPTH_TEST);

	// Create program.
	static const char* progId = "App::blitSMToScreen";
	GLContext::Program* prog = gl->getProgram(progId);
	if (!prog)
	{
		prog = new GLContext::Program(
			FW_GL_SHADER_SOURCE(
			attribute vec4 posAttrib;
		attribute vec2 texAttrib;
		varying vec2 uv;
		void main()
		{
			gl_Position = posAttrib;
			uv = texAttrib;
		}
		),
			FW_GL_SHADER_SOURCE(
			uniform sampler2D texSampler;
		varying vec2 uv;
		uniform float tonemapBoost;
		uniform float multiplier;

		void main()
		{
			// Transform depth from [0, 1] to NDC's [-1, 1].
			float depth = 2.0 * texture2D(texSampler, vec2(uv.x, 1.0-uv.y)).r - 1.0;

			// Map to linear z, assuming infinite far plane.
			float nearDepth = multiplier;
			float val = 2.0 * nearDepth / (1.0 - depth);

			// Gamma correction.
			val = pow(max(val, 0.0), 1.0 / 2.2);

			gl_FragColor = vec4(vec3(val), 1.0);
		}
		));
		gl->setProgram(progId, prog);
	}

	prog->use();

	gl->setAttrib(prog->getAttribLoc("posAttrib"), 4, GL_FLOAT, 0, posAttrib);
	gl->setAttrib(prog->getAttribLoc("texAttrib"), 2, GL_FLOAT, 0, texAttrib);
	gl->setUniform(prog->getUniformLoc("multiplier"), m_shadowMapVisMultiplier);

	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, ls.getShadowTextureHandle());
	gl->setUniform(prog->getUniformLoc("texSampler"), 0);

	glDrawArrays(GL_TRIANGLE_STRIP, 0, 4);
}



//------------------------------------------------------------------------


void App::loadMesh(const String& fileName)
{

	// find the scene name; the file path without preceding folders and file extension

	std::string path = std::string(fileName.getPtr());

	size_t begin = path.find_last_of("/\\")+1,
		     end = path.find_last_of(".");

	m_results.scene_name = path.substr(begin, end - begin);

	std::cout << "Scene name: " << m_results.scene_name << std::endl;

	m_window.showModalMessage(sprintf("Loading mesh from '%s'...", fileName.getPtr()));
	String oldError = clearError();
	std::unique_ptr<MeshBase> mesh((MeshBase*)importMesh(fileName));

	String newError = getError();

	if (restoreError(oldError))
	{
		m_commonCtrl.message(sprintf("Error while loading '%s': %s", fileName.getPtr(), newError.getPtr()));
		return;
	}

	m_meshFileName = fileName;

	// convert input mesh to colored format
	m_mesh.reset(new MeshWithColors(*mesh));
	


	// fix input colors to white so we see something
	for ( S32 i = 0; i < m_mesh->numVertices(); ++i )
		m_mesh->mutableVertex(i).c = Vec3f(1, 1, 1);

	m_commonCtrl.message(sprintf("Loaded mesh from '%s'", fileName.getPtr()));

	// build the BVH!
	constructTracer();
}

//------------------------------------------------------------------------

void App::saveMesh(const String& fileName)
{
	if (!m_mesh)
	{
		m_commonCtrl.message("No mesh to save!");
		return;
	}

	m_window.showModalMessage(sprintf("Saving mesh to '%s'...", fileName.getPtr()));
	String oldError = clearError();
	exportMesh(fileName, m_mesh.get());
	String newError = getError();

	if (restoreError(oldError))
	{
		m_commonCtrl.message(sprintf("Error while saving '%s': %s", fileName.getPtr(), newError.getPtr()));
		return;
	}

	m_meshFileName = fileName;
	m_commonCtrl.message(sprintf("Saved mesh to '%s'", fileName.getPtr()));
}

//------------------------------------------------------------------------

// This function iterates over all the "sub-meshes" (parts of the object with different materials),
// heaps all the vertices and triangles together, and calls the BVH constructor.
// It is the responsibility of the tree to free the data when deleted.
// This functionality is _not_ part of the RayTracer class in order to keep it separate
// from the specifics of the Mesh class.
void App::constructTracer()
{
	// fetch vertex and triangle data ----->
	m_rtTriangles.clear();
	m_rtTriangles.reserve(m_mesh->numTriangles());


	for (int i = 0; i < m_mesh->numSubmeshes(); ++i)
	{
		const Array<Vec3i>& idx = m_mesh->indices(i);
		for (int j = 0; j < idx.getSize(); ++j)
		{

			const VertexPNTC &v0 = m_mesh->vertex(idx[j][0]),
						     &v1 = m_mesh->vertex(idx[j][1]),
							 &v2 = m_mesh->vertex(idx[j][2]);

			RTTriangle t = RTTriangle(v0, v1, v2);

			t.m_data.vertex_indices = idx[j];
			t.m_material = &(m_mesh->material(i));

			m_rtTriangles.push_back(t);
		}
	}


	// compute checksum

	m_rtVertexPositions.clear();
	m_rtVertexPositions.reserve(m_mesh->numVertices());
	for (int i = 0; i < m_mesh->numVertices(); ++i)
		m_rtVertexPositions.push_back(m_mesh->vertex(i).p);

	String md5 = RayTracer::computeMD5(m_rtVertexPositions);
	FW::printf("Mesh MD5: %s\n", md5.getPtr());

	// construct a new ray tracer (deletes the old one if there was one)
	m_rt.reset(new RayTracer());

	// whether we want to try loading a saved hierarchy from disk
	bool tryLoadHierarchy = false;

	// always construct when measuring performance
	if (m_settings.batch_render)
		tryLoadHierarchy = false;

	if (tryLoadHierarchy)
	{
		// check if saved hierarchy exists

		std::string meshName = m_meshFileName.getPtr();
		std::string hierarchyName = meshName.substr(0, meshName.find_last_of("."));
#ifdef _WIN64
		hierarchyName += "_x64";
#endif
		
		hierarchyName += ".hierarchy";

		String hierarchyCacheFile = hierarchyName.c_str();

		if (fileExists(hierarchyCacheFile.getPtr()))
		{
			// yes, load!
			m_rt->loadHierarchy(hierarchyCacheFile.getPtr(), m_rtTriangles);
			::printf("Loaded hierarchy from %s\n", hierarchyCacheFile.getPtr());
		}
		else
		{
			// no, construct...
			LARGE_INTEGER start, stop, frequency;
			QueryPerformanceFrequency(&frequency);
			QueryPerformanceCounter(&start); // Start time stamp		

			m_rt->constructHierarchy(m_rtTriangles, m_settings.splitMode);

			QueryPerformanceCounter(&stop); // Stop time stamp

			int build_time = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds
			std::cout << "Build time: " << build_time << " ms" << std::endl;
			// .. and save!
			m_rt->saveHierarchy(hierarchyCacheFile.getPtr(), m_rtTriangles);
			::printf("Saved hierarchy to %s\n", hierarchyCacheFile.getPtr());
		}
	}
	else
	{
		// nope, bite the bullet and construct it

		LARGE_INTEGER start, stop, frequency;
		QueryPerformanceFrequency(&frequency);
		QueryPerformanceCounter(&start); // Start time stamp		
		
		m_rt->constructHierarchy(m_rtTriangles, m_settings.splitMode);

		QueryPerformanceCounter(&stop); // Stop time stamp

		m_results.build_time = (int)((stop.QuadPart - start.QuadPart) * 1000.0 / frequency.QuadPart); // Get timer result in milliseconds
		std::cout << "Build time: " << m_results.build_time << " ms"<< std::endl;
	}

}



//------------------------------------------------------------------------

void App::downscaleTextures(MeshBase* mesh)
{
	FW_ASSERT(mesh);
	Hash<const Image*, Texture> hash;
	for (int submeshIdx = 0; submeshIdx < mesh->numSubmeshes(); submeshIdx++)
		for (int textureIdx = 0; textureIdx < MeshBase::TextureType_Max; textureIdx++)
		{
		Texture& tex = mesh->material(submeshIdx).textures[textureIdx];
		if (tex.exists())
		{
			const Image* orig = tex.getImage();
			if (!hash.contains(orig))
			{
				Image* scaled = orig->downscale2x();
				hash.add(orig, (scaled) ? Texture(scaled, tex.getID()) : tex);
			}
			tex = hash.get(orig);
		}
		}
}

//------------------------------------------------------------------------

void App::chopBehindPlane(MeshBase* mesh, const Vec4f& pleq)
{
	FW_ASSERT(mesh);
	int posAttrib = mesh->findAttrib(MeshBase::AttribType_Position);
	if (posAttrib == -1)
		return;

	for (int submeshIdx = 0; submeshIdx < mesh->numSubmeshes(); submeshIdx++)
	{
		Array<Vec3i>& inds = mesh->mutableIndices(submeshIdx);
		int triOut = 0;
		for (int triIn = 0; triIn < inds.getSize(); triIn++)
		{
			if (dot(mesh->getVertexAttrib(inds[triIn].x, posAttrib), pleq) >= 0.0f ||
				dot(mesh->getVertexAttrib(inds[triIn].y, posAttrib), pleq) >= 0.0f ||
				dot(mesh->getVertexAttrib(inds[triIn].z, posAttrib), pleq) >= 0.0f)
			{
				inds[triOut++] = inds[triIn];
			}
		}
		inds.resize(triOut);
	}

	mesh->clean();
}

//------------------------------------------------------------------------

// Dummy shader, will be overwritten by the shadow program
void App::setupShaders( GLContext* gl )
{
	GLContext::Program* prog = new GLContext::Program(
		"#version 120\n"
		FW_GL_SHADER_SOURCE(
		uniform mat4 posToClip;
	uniform mat4 posToCamera;

	void main()
	{
		vec4 pos = vec4(positionAttrib, 1.0);
		gl_Position = posToClip * pos;
	}
	),
		"#version 120\n"
		FW_GL_SHADER_SOURCE(
		void main()
	{
		gl_FragColor = vec4(0.1, 0.3, 0.7, 1.0);
	}
	));

	gl->setProgram( "MeshBase::draw_generic", prog );
}


void App::setupRenderToTexture(const Vec2i& resolution)
{
	// Create a framebuffer object that holds the off-screen rendering buffers
	glGenFramebuffers(1, &m_rttFBO);
	glBindFramebuffer(GL_FRAMEBUFFER, m_rttFBO);
	GLContext::checkErrors();

	// Create and attach a renderbuffer for the depth map
	glGenRenderbuffers(1, &m_rttDepth);
	glBindRenderbuffer(GL_RENDERBUFFER, m_rttFBO);
	glRenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT, resolution.x, resolution.y);
	glFramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, m_rttDepth);
	GLContext::checkErrors();

	// Create and attach the render target texture in 16-bit floating point format
	glGenTextures(1, &m_rttTex);
	glBindTexture(GL_TEXTURE_2D, m_rttTex);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA16F, resolution.x, resolution.y,	0, GL_RGBA, GL_FLOAT, NULL);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
	glTexParameteri (GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
	glFramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, m_rttTex, 0);
	GLContext::checkErrors();

	// Detach the FBO for now
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
}

void App::deleteRenderToTexture()
{
	glBindFramebuffer(GL_FRAMEBUFFER, 0);
	glDeleteFramebuffers(1, &m_rttFBO);
	glDeleteTextures(1, &m_rttTex);
	glDeleteRenderbuffers(1, &m_rttDepth);

	GLContext::checkErrors();
}


//------------------------------------------------------------------------

void FW::init(std::vector<std::string>& args)
{
	new App(args);
}

//------------------------------------------------------------------------

bool App::fileExists(const String& fn)
{
	FILE* pF = fopen(fn.getPtr(), "rb");
	if (pF != 0)
	{
		fclose(pF);
		return true;
	}
	else
	{
		return false;
	}
}

//------------------------------------------------------------------------
