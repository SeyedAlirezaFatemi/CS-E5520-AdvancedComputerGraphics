#include "AreaLight.hpp"

#include <iostream>

namespace FW {

void AreaLight::draw(const Mat4f& worldToCamera, const Mat4f& projection) {
    glUseProgram(0);
    glMatrixMode(GL_PROJECTION);
    glLoadMatrixf((float*)&projection);
    glMatrixMode(GL_MODELVIEW);
    Mat4f S = Mat4f::scale(Vec3f(m_size, 1));
    Mat4f M = worldToCamera * m_xform * S;
    glLoadMatrixf((float*)&M);
    glBegin(GL_TRIANGLES);
    glColor3fv(&m_E.x);
    glVertex3f(1, 1, 0);
    glVertex3f(1, -1, 0);
    glVertex3f(-1, -1, 0);
    glVertex3f(1, 1, 0);
    glVertex3f(-1, -1, 0);
    glVertex3f(-1, 1, 0);
    glEnd();
}

void AreaLight::sample(float& pdf, Vec3f& p, int index, Random& rnd) {
    // YOUR CODE HERE (R2):
    // You should draw a random point on the light source and evaluate the PDF.
    // Store the results in "pdf" and "p".
    //
    // Note: The "size" member is _one half_ the diagonal of the light source.
    // That is, when you map the square [-1,1]^2 through the scaling matrix
    //
    // S = ( size.x    0   )
    //     (   0    size.y )
    //
    // the result is the desired light source quad (see draw() function above).
    // This means the total area of the light source is 4*size.x*size.y.
    // This has implications for the computation of the PDF.

    // For extra credit, implement QMC sampling using some suitable sequence.
    // Use the "index" input for controlling the progression of the sequence from
    // the outside. If you only implement purely random sampling, "index" is not required.

    pdf = 1.f / (4.f * m_size.x * m_size.y);

    // auto x = rnd.getF32(-1.f, 1.f);
    // auto y = rnd.getF32(-1.f, 1.f);

    auto x = sobol::sample(index, 0) * 2 - 1;
    auto y = sobol::sample(index, 1) * 2 - 1;

    Mat4f S = Mat4f::scale(Vec3f(m_size, 1));
    p = m_xform * S * Vec3f(x, y, 0.f);
}

}  // namespace FW
