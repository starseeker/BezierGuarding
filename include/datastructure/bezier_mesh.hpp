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

    std::vector<int> allctrlpts;  // All controlpoints (includes offset for non vertex)
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

    vector< Vec2<T> > allPts;
    std::vector<int> contrlPts( Triangle t);
    std::vector< std::vector<Triangle> > neighbourFaces;
    std::vector< std::set<int> > neighbourFaces_fixedV;
    std::vector< std::map<int, int> > localVMap;

    std::vector< std::pair<double,double> > uv_sampling;
    std::vector< std::pair<int, int> > C_IJK, C1_IJK;
    std::vector< std::vector<T> > UV_IJK;

    int m_control_point_count;
    T bz_pow(double u, int n);
    T bz_factorial(int n);
    T bz_value(int degree, int i, int j, int k, double u, double v);
    int bz_control_point_local_id(int i, int j, int k);
    T bz_get_uv_ijk(int ui, int i, int j, int k);
    std::vector< std::vector<T> > bz_jacobian(Triangle& t);
    std::vector< std::vector<T> > bz_jacobian(std::vector< Vec2<T> >& bz_vt, std::vector<int>& ctrl_pt);

    void bz_gradient(std::vector<int>& ctrl_pt, std::vector< std::vector<T> >& Jac, std::vector< Vec2<T> >& Grad, std::map<int, int>& map );
    T bz_energy(std::vector< std::vector<T> >& Jac);
    T bz_energy(vector< Vec2<T> >& CtrPt_temp, int v_id);
    T m_lambda;

    bool check_flip(vector< Vec2<T> >& ctrPt, std::vector<int>& f_ctrPt);
    bool check_flip(vector< Vec2<T> >& ctrPt, int v_id, int f_id=-1);


    void fillNeighbourInfo();
    void optimization_conformal();
    void gradient_step( int v_id);
};

} // namespace bzmsh

#endif
