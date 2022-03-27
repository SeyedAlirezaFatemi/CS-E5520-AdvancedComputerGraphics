#pragma once

#include <base/Main.hpp>
#include <io/StateDump.hpp>

#include <base/Math.hpp>
#include <base/Random.hpp>
#include <3d/Mesh.hpp>

#include <vector>

#include "RayTracer.hpp"

namespace FW
{


//------------------------------------------------------------------------

typedef Mesh<VertexPNTC>	MeshWithColors;

//------------------------------------------------------------------------


class GLContext;

// This is a technical thing that holds the off-screen buffers related
// to shadow map rendering; no need to touch this for the standard requirements.
class ShadowMapContext
{
public:
	ShadowMapContext() :
	  m_depthRenderbuffer(0),
	  m_framebuffer(0),
	  m_resolution(256, 256)
		{ }

	~ShadowMapContext() { teardown(); }

	bool setup(Vec2i resolution);
	void teardown() {};

	GLuint allocateDepthTexture();
	void attach(GLContext *gl, GLuint texture);

protected:
	GLuint m_depthRenderbuffer;
	GLuint m_framebuffer;
	Vec2i m_resolution;
};


class LightSource
{
public:
	LightSource() : 
	  m_E(1,1,1), 
	  m_size(0.125f,0.125f), 
	  m_fov(20.0f), 
	  m_near(0.01f), 
	  m_far(100.0f),
	  m_shadowMapTexture(0),
	  m_enabled(true)
	  { }

	Vec3f			getPosition(void) const			{ return Vec4f(m_xform.getCol(3)).getXYZ(); }
	void			setPosition(const Vec3f& p)		{ m_xform.setCol(3, Vec4f(p, 1.0f)); }

	Mat3f			getOrientation(void) const		{ return m_xform.getXYZ(); }
	void			setOrientation(const Mat3f& R)	{ m_xform.setCol(0,Vec4f(R.getCol(0),0.0f)); m_xform.setCol(1,Vec4f(R.getCol(1),0.0f)); m_xform.setCol(2,Vec4f(R.getCol(2),0.0f)); }

	Vec3f			getNormal(void) const			{ return -Vec4f(m_xform.getCol(2)).getXYZ(); }

	Vec2f			getSize(void) const				{ return m_size; }
	void			setSize(const Vec2f& s)			{ m_size = s; }

	Vec3f			getEmission(void) const			{ return m_E; }
	void			setEmission(const Vec3f& E)		{ m_E = E; }

	float			getFOV(void) const				{ return m_fov; }
	float			getFOVRad(void) const			{ return m_fov * 3.1415926f / 180.0f; }
	void			setFOV(float fov)				{ m_fov = fov; }

	float			getFar(void) const				{ return m_far; }
	void			setFar(float f)					{ m_far = f; }

	float			getNear(void) const				{ return m_near; }
	void			setNear(float n)				{ m_near = n; }

	
	void			draw( const Mat4f& worldToCamera, const Mat4f& projection, bool show_axis = false, bool show_frame = false, bool show_square = false); // for visualization

	void			readState( StateDump& d )			{ d.pushOwner("areaLight"); d.get(m_xform,"xform"); d.get(m_size,"size"); d.get(m_E,"E"); d.popOwner(); }
	void			writeState( StateDump& d ) const	{ d.pushOwner("areaLight"); d.set(m_xform,"xform"); d.set(m_size,"size"); d.set(m_E,"E"); d.popOwner(); }

	void			setEnabled(bool e) { m_enabled = e; };
	bool			isEnabled() const { return m_enabled; }

	void			renderShadowedScene(GLContext *gl, MeshWithColors *scene, const Mat4f& worldToCamera, const Mat4f& projection, bool fromLight = false);
	void			renderShadowMap(FW::GLContext *gl, MeshWithColors *scene, ShadowMapContext *sm, bool debug = false);

	Mat4f			getPosToLightClip() const;

	void			sampleEmittedRays(int num, std::vector<Vec3f>& origs, std::vector<Vec3f>& dirs, std::vector<Vec3f>& E_times_pdf) const;

	// OpenGL stuff:
	GLuint			getShadowTextureHandle() const { return m_shadowMapTexture; }
	void			freeShadowMap() { glDeleteTextures(1, &m_shadowMapTexture); m_shadowMapTexture = 0; }
protected:
	static void		renderSceneRaw(FW::GLContext *gl, MeshWithColors *scene, GLContext::Program *prog);

	Mat4f	m_xform;	// encodes position and orientation in world space
	Vec2f	m_size;		// physical size of the emitting surface (ignored in the basic implementation)
	Vec3f	m_E;		// diffuse emission (W/m^2)
	float	m_fov;		// field of view in degrees
	float	m_near;     // near clipping distance
	float	m_far;      // far clipping distance

	bool	m_enabled;  // Is the light on, i.e. will we bother to render with it?
	GLuint	m_shadowMapTexture; // OpenGL texture handle

};

}
