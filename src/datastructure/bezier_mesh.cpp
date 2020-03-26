#include "datastructure/bezier_mesh.hpp"

#include "datastructure/bbox.hpp"
#include "datastructure/curve.hpp"
#include "datastructure/guarded_bezier_curve.hpp"
#include "datastructure/triangulation_constraints.hpp"
#include "util/helper_functions.hpp"
#include <map>

namespace bzmsh
{

template <typename T>
BezierMesh<T>::BezierMesh(int _degree) : degree(_degree)
{
}

template <typename T>
int BezierMesh<T>::addVertex(const Vec2<T>& v)
{
    if (vertex2index.find(v) == vertex2index.end())
    {
        vertex2index[v] = allVertices.size();
        allVertices.emplace_back(v);
    }
    return vertex2index[v];
}

template <typename T>
int BezierMesh<T>::addCtrlpt(const Vec2<T>& v)
{
    if (ctrlpt2index.find(v) == ctrlpt2index.end())
    {
        ctrlpt2index[v] = allCtrlpts.size();
        allCtrlpts.emplace_back(v);
    }
    return ctrlpt2index[v];
}

template <typename T>
int BezierMesh<T>::addEdge(int from, int to, int marker)
{
    Edge e(from, to, marker);
    if (edge2index.find(e) == edge2index.end())
    {
        edge2index[e] = allEdges.size();
        allEdges.emplace_back(e);
    }
    return edge2index[e];
}

template <typename T>
int BezierMesh<T>::addEdge(const Vec2<T>& vfrom, const Vec2<T>& vto, int marker)
{
    int from = addVertex(vfrom);
    int to = addVertex(vto);
    Edge e(from, to, marker);
    if (edge2index.find(e) == edge2index.end())
    {
        edge2index[e] = allEdges.size();
        allEdges.emplace_back(e);
    }
    return edge2index[e];
}

template <typename T>
int BezierMesh<T>::addTriangle(const vector<int>& vertexIndices)
{
    assert(vertexIndices.size() == 3);
    Triangle tri;
    tri.vertices = vertexIndices;

    for (int i = 0; i < 3; i++)
    {
        tri.edges.emplace_back(addEdge(tri.vertices[i], tri.vertices[(i + 1) % 3], -2));
    }

    allTriangles.emplace_back(tri);

    return allTriangles.size() - 1;
}

template <typename T>
int BezierMesh<T>::addTriangle(const vector<Vec2<T>>& vertices)
{
    assert(vertices.size() == 3);
    vector<int> vertexIndices;
    for (int i = 0; i < 3; i++)
    {
        vertexIndices.emplace_back(addVertex(vertices[i]));
    }
    return addTriangle(vertexIndices);
}

template struct BezierMesh<double>;
template struct BezierMesh<mpq_class>;

} // namespace bzmsh
