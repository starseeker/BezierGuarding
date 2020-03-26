#ifndef BZMSH_BEZIERMESH_HPP
#define BZMSH_BEZIERMESH_HPP

#include "datastructure/bbox.hpp"
#include "datastructure/curve.hpp"
#include "datastructure/guarded_bezier_curve.hpp"
#include "datastructure/triangulation_constraints.hpp"
#include "util/helper_functions.hpp"
#include <map>

namespace bzmsh
{
using std::map;

/**
 * @brief Undirected edge between 2 vertices. Can be marked.
 */
struct Edge
{
    /**
     * @brief Mark for bbox edge
     */
    const static int MARK_BBOX = -5;
    /**
     * @brief Mark for edge from subdivision (for eps visualization)
     */
    const static int MARK_SUBDIV = -4;
    /**
     * @brief Mark for center triangle edges in subdivision (for eps visualization)
     */
    const static int MARK_SUBDIV_CENTER = -3;
    /**
     * @brief Default marker. Standard edge. Non-boundary. Non-curved.
     */
    const static int MARK_DEFAULT = -2;
    /**
     * @brief Boundary edge. Non-curved.
     */
    const static int MARK_BOUNDARY = -1;
    /**
     * @brief IDs >= this will refer to the curve that this edge represents.
     */
    const static int MARK_CURVEIDLIMIT = 0;

    /**
     * @brief Construct a new edge
     *
     * @param _from vertex index 1
     * @param _to vertex inex 2
     * @param _marker marker to assign
     */
    Edge(int _from, int _to, int _marker = MARK_DEFAULT) : from(_from), to(_to), marker(_marker)
    {
        if (from == to)
        {
            Logger::lout(Logger::ERROR) << "invalid edge created" << endl;
            from = -1;
            to = -1;
        }
        if (from > to)
        {
            std::swap(from, to);
        }
    }

    int from = -1;
    int to = -1;
    int marker = MARK_DEFAULT;

    /**
     * @brief Edge ctrlpt indices. Do not include the endpoints!
     * Therefore number of controlpoints = degree -1
     */
    vector<int> ctrlpts;

    /**
     * @brief Comparator for ordering edges based on first vertex
     */
    struct compare
    {
        bool operator()(const Edge& lhs, const Edge& rhs) const
        {
            return (lhs.from < rhs.from) || ((lhs.from == rhs.from) && (lhs.to < rhs.to));
        }
    };
};

struct Triangle
{
    vector<int> vertices; // counterclockwise
    vector<int> edges;    // counterclockwise
    vector<int> ctrlpts;  // only inner controlpoints!
};

template<typename T>
struct BezierMesh
{
    BezierMesh(int _degree = -1);

    /**
     * @brief Add a vertex into the mesh
     *
     * @param v position vector
     * @return int index of vertex in allVertices vector
     */
    int addVertex(const Vec2<T>& v);

    /**
     * @brief Add a control point into the mesh
     *
     * @param v position vector
     * @return int index of control point in allCtrlpts vector
     */
    int addCtrlpt(const Vec2<T>& v);

    /**
     * @brief Add an edge into the mesh
     *
     * @param from index of first vertex
     * @param to index of second vertex
     * @param marker marker to assign to the edge
     * @return int index of edge in allEdges vector
     */
    int addEdge(int from, int to, int marker);

    /**
     * @brief Add an edge into the mesh
     *
     * @param vfrom first vertex
     * @param vto second vertex
     * @param marker marker to assign to the edge
     * @return int index of edge in allEdges vector
     */
    int addEdge(const Vec2<T>& vfrom, const Vec2<T>& vto, int marker);

    /**
     * @brief Add a triangle into the mesh
     *
     * @param vertexIndices the three vertex indices in counterclockwise order
     * @return int index of the triangle in allTriangles vector
     */
    int addTriangle(const vector<int>& vertexIndices);

    /**
     * @brief Add a triangle into the mesh
     *
     * @param vertices the three vertices in ccw order
     * @return int index of the triangle in allTriangles vector
     */
    int addTriangle(const vector<Vec2<T>>& vertices);

    int triangle_count() { return  allTriangles.size(); };

    int degree;

    map<struct Vec2<T>, int, struct Vec2<T>::compareLex> vertex2index;
    map<struct Vec2<T>, int, struct Vec2<T>::compareLex> ctrlpt2index;
    map<Edge, int, Edge::compare> edge2index;
    vector<Vec2<T>> allVertices;
    vector<Vec2<T>> allCtrlpts;
    vector<Edge> allEdges;
    vector<Triangle> allTriangles;
    
    vector<bool> ctrlpointFlipped; //remove
};

} // namespace bzmsh

#endif
