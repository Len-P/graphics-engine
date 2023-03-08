#include "Figure3D.h"

Figure3D::Figure::Figure(vector<Vector3D> &aPoints, vector<Face> &aFaces, const Color &aColor)
{
    points = aPoints;
    faces = aFaces;
    color = aColor;
}
