/*
 * Copyright (c) 2011, Georgia Tech Research Corporation
 * All rights reserved.
 *
 * Author(s): Jeongseok Lee <jslee02@gmail.com>
 * Date: 05/23/2013
 *
 * Geoorgia Tech Graphics Lab and Humanoid Robotics Lab
 *
 * Directed by Prof. C. Karen Liu and Prof. Mike Stilman
 * <karenliu@cc.gatech.edu> <mstilman@cc.gatech.edu>
 *
 * This file is provided under the following "BSD-style" License:
 *   Redistribution and use in source and binary forms, with or
 *   without modification, are permitted provided that the following
 *   conditions are met:
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 *   CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES,
 *   INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 *   MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 *   CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF
 *   USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 *   AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *   LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *   ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *   POSSIBILITY OF SUCH DAMAGE.
 */

#include <iostream>
#include <gtest/gtest.h>
#include <boost/graph/adjacency_matrix.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/depth_first_search.hpp>
#include <boost/range/irange.hpp>
#include <boost/pending/indirect_cmp.hpp>

#include "common/UtilsCode.h"
#include "math/Geometry.h"
#include "math/Helpers.h"
#include "dynamics/BallJoint.h"
#include "dynamics/RevoluteJoint.h"
#include "dynamics/FreeJoint.h"
#include "dynamics/TranslationalJoint.h"
#include "dynamics/Skeleton.h"
#include "simulation/World.h"

#include "TestHelpers.h"


using namespace dart;
using namespace math;
using namespace dynamics;
using namespace simulation;

using namespace boost;

#define SKELETON_TOL 0.01

/******************************************************************************/
class JOINTS : public testing::Test
{
public:
//    void kinematicsTest(Joint* _joint);
};

template <class VertexListGraph, class Name>
void print_vertices(const VertexListGraph& G, Name name)
{
    typename boost::graph_traits<VertexListGraph>::vertex_iterator vi, vi_end;
    for (boost::tie(vi,vi_end) = vertices(G); vi != vi_end; ++vi)
    {
        std::cout << get(name, *vi) << " ";
    }
    std::cout << std::endl;
}

template <class EdgeListGraph, class Name>
void print_edges(const EdgeListGraph& G, Name name)
{
    typename boost::graph_traits<EdgeListGraph>::edge_iterator ei, ei_end;
    for (boost::tie(ei, ei_end) = edges(G); ei != ei_end; ++ei)
    {
        std::cout << "(" << get(name, source(*ei, G))
                  << ", " << get(name, target(*ei, G)) << ") ";
    }
    std::cout << std::endl;
}

/******************************************************************************/

//TEST(SKELETON, GRAPH)
//{
//    using namespace boost;

//    const char* name("ABCDE");

//    typedef boost::adjacency_matrix<boost::directedS> MatrixGraph;
//    typedef boost::graph_traits<MatrixGraph>::vertex_descriptor VertexDescriptor;
//    typedef boost::graph_traits<MatrixGraph>::edge_descriptor EdgeDescriptor;
//    typedef std::pair<BodyNode*, BodyNode*> Edge;

//    MatrixGraph ug(5);

//    std::vector<BodyNode*> bodies;

//    BodyNode b0;
//    BodyNode b1;
//    BodyNode b2;
//    BodyNode b3;
//    BodyNode b4;

//    bodies.push_back(&b0);
//    bodies.push_back(&b1);
//    bodies.push_back(&b2);
//    bodies.push_back(&b3);
//    bodies.push_back(&b4);

////    boost::add_vertex(0, ug);
////    boost::add_vertex(1, ug);
////    boost::add_vertex(2, ug);
////    boost::add_vertex(3, ug);
////    boost::add_vertex(4, ug);

//    boost::add_edge(0, 1, ug);
//    boost::add_edge(1, 2, ug);
//    boost::add_edge(2, 0, ug);
//    boost::add_edge(2, 3, ug);
//    boost::add_edge(3, 4, ug);


////    VertexDescriptor b0;
////    VertexDescriptor b1;
////    VertexDescriptor b2;
////    VertexDescriptor b3;
////    VertexDescriptor b4;

////    boost::add_edge(b0, b1, ug);
////    boost::add_edge(b1, b2, ug);
////    boost::add_edge(b2, b0, ug);
////    boost::add_edge(b2, b3, ug);
////    boost::add_edge(b3, b4, ug);

//    //boost::put(boost::vertex_name, ug, b0, "BodyNode1");
//    print_vertices(ug, name);
//    print_edges(ug, name);

//    write_graphviz(cout, ug);
//}


//TEST(SKELETON, GRAPH2)
//{
//    using namespace boost;

//    /* define the graph type
//      listS: selects the STL list container to store
//             the OutEdge list
//      vecS: selects the STL vector container to store
//            the vertices
//      directedS: selects directed edges
//*/
//    typedef adjacency_list< listS, vecS, directedS > digraph;

//    // instantiate a digraph object with 8 vertices
//    digraph g(8);

//    // add some edges
//    add_edge(0, 1, g);
//    add_edge(1, 5, g);
//    add_edge(5, 6, g);
//    add_edge(2, 3, g);
//    add_edge(2, 4, g);
//    add_edge(3, 5, g);
//    add_edge(4, 5, g);
//    add_edge(5, 7, g);

//    // represent graph in DOT format and send to cout
//    write_graphviz(cout, g);
//}


template < typename TimeMap >
class dfs_time_visitor : public default_dfs_visitor {
  typedef typename property_traits < TimeMap >::value_type T;
public:
  dfs_time_visitor(TimeMap dmap, TimeMap fmap, T & t)
:  m_dtimemap(dmap), m_ftimemap(fmap), m_time(t) {
  }
  template < typename Vertex, typename Graph >
    void discover_vertex(Vertex u, const Graph & g) const
  {
    put(m_dtimemap, u, m_time++);
  }
  template < typename Vertex, typename Graph >
    void finish_vertex(Vertex u, const Graph & g) const
  {
    put(m_ftimemap, u, m_time++);
  }
  TimeMap m_dtimemap;
  TimeMap m_ftimemap;
  T & m_time;
};

TEST(SKELETON, GRAPH3)
{
    // Select the graph type we wish to use
    typedef adjacency_list < vecS, vecS, directedS > graph_t;
    typedef graph_traits < graph_t >::vertices_size_type size_type;
    // Set up the vertex names
    enum
    { u, v, w, x, y, z, N };
    char name[] = { 'u', 'v', 'w', 'x', 'y', 'z' };
    // Specify the edges in the graph
    typedef std::pair < int, int >E;
    E edge_array[] = { E(u, v), E(u, x), E(x, v), E(y, x),
                       E(v, y), E(w, y), E(w, z), E(z, z)
                     };
#if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
    graph_t g(N);
    for (std::size_t j = 0; j < sizeof(edge_array) / sizeof(E); ++j)
        add_edge(edge_array[j].first, edge_array[j].second, g);
#else
    graph_t g(edge_array, edge_array + sizeof(edge_array) / sizeof(E), N);
#endif

    // Typedefs
    typedef boost::graph_traits < graph_t >::vertex_descriptor Vertex;
    typedef size_type* Iiter;

    // discover time and finish time properties
    std::vector < size_type > dtime(num_vertices(g));
    std::vector < size_type > ftime(num_vertices(g));
    size_type t = 0;
    dfs_time_visitor < size_type * >vis(&dtime[0], &ftime[0], t);

    depth_first_search(g, visitor(vis));

    // use std::sort to order the vertices by their discover time
    std::vector < size_type > discover_order(N);
    integer_range < size_type > r(0, N);
    std::copy(r.begin(), r.end(), discover_order.begin());
    std::sort(discover_order.begin(), discover_order.end(),
              indirect_cmp < Iiter, std::less < size_type > >(&dtime[0]));
    std::cout << "order of discovery: ";
    int i;
    for (i = 0; i < N; ++i)
        std::cout << name[discover_order[i]] << " ";

    std::vector < size_type > finish_order(N);
    std::copy(r.begin(), r.end(), finish_order.begin());
    std::sort(finish_order.begin(), finish_order.end(),
              indirect_cmp < Iiter, std::less < size_type > >(&ftime[0]));
    std::cout << std::endl << "order of finish: ";
    for (i = 0; i < N; ++i)
        std::cout << name[finish_order[i]] << " ";
    std::cout << std::endl;

}


//using namespace boost;
//template < typename TimeMap > class bfs_time_visitor:public default_bfs_visitor {
//  typedef typename property_traits < TimeMap >::value_type T;
//public:
//  bfs_time_visitor(TimeMap tmap, T & t):m_timemap(tmap), m_time(t) { }
//  template < typename Vertex, typename Graph >
//    void discover_vertex(Vertex u, const Graph & g) const
//  {
//    put(m_timemap, u, m_time++);
//  }
//  TimeMap m_timemap;
//  T & m_time;
//};

//TEST(SKELETON, GRAPH4)
//{
//    // Select the graph type we wish to use
//    typedef adjacency_list < vecS, vecS, directedS > graph_t;
//    // Set up the vertex IDs and names
//    enum { r, s, t, u, v, w, x, y, N };
//    const char *name = "rstuvwxy";
//    // Specify the edges in the graph
//    typedef std::pair < int, int >E;
//    E edge_array[] = { E(r, s), E(r, v), E(s, w), E(w, r), E(w, t),
//      E(w, x), E(x, t), E(t, u), E(x, y), E(u, y)
//    };
//    // Create the graph object
//    const int n_edges = sizeof(edge_array) / sizeof(E);
//  #if defined(BOOST_MSVC) && BOOST_MSVC <= 1300
//    // VC++ has trouble with the edge iterator constructor
//    graph_t g(N);
//    for (std::size_t j = 0; j < n_edges; ++j)
//      add_edge(edge_array[j].first, edge_array[j].second, g);
//  #else
//    typedef graph_traits<graph_t>::vertices_size_type v_size_t;
//    graph_t g(edge_array, edge_array + n_edges, v_size_t(N));
//  #endif

//    // Typedefs
//    typedef graph_traits < graph_t >::vertex_descriptor Vertex;
//    typedef graph_traits < graph_t >::vertices_size_type Size;
//    typedef Size* Iiter;

//    // a vector to hold the discover time property for each vertex
//    std::vector < Size > dtime(num_vertices(g));

//    Size time = 0;
//    bfs_time_visitor < Size * >vis(&dtime[0], time);
//    breadth_first_search(g, vertex(s, g), visitor(vis));

//    // Use std::sort to order the vertices by their discover time
//    std::vector<graph_traits<graph_t>::vertices_size_type > discover_order(N);
//    integer_range < int >range(0, N);
//    std::copy(range.begin(), range.end(), discover_order.begin());
//    std::sort(discover_order.begin(), discover_order.end(),
//              indirect_cmp < Iiter, std::less < Size > >(&dtime[0]));

//    std::cout << "order of discovery: ";
//    for (int i = 0; i < N; ++i)
//      std::cout << name[discover_order[i]] << " ";
//    std::cout << std::endl;
//}

/******************************************************************************/
int main(int argc, char* argv[])
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}


