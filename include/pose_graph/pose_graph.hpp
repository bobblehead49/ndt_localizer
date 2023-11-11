// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#ifndef POSE_GRAPH_HPP_
#define POSE_GRAPH_HPP_

#include <unordered_map>
#include <unordered_set>
#include <vector>


class PoseGraph
{
private:
    std::unordered_map<int, std::unordered_set<int>> adjacency_list;

public:
    /**
     * \brief Constructor
     */
    PoseGraph() {}

    /**
     * \brief Destructor
     */
    ~PoseGraph() {}

    /**
     * \brief Add an edge to the graph.
     * \param a The first node.
     * \param b The second node.
     */
    void add_edge(int a, int b);

    /**
     * \brief Remove a node from the graph.
     * \param a The node to remove.
     */
    void remove_node(int a);

    /**
     * \brief Check if an edge exists between two nodes.
     * \param a The first node.
     * \param b The second node.
     * \return True if an edge exists between the two nodes, false otherwise.
     */
    bool has_edge(int a, int b);

    /**
     * \brief Check if a node exists in the graph.
     * \param node The node to check.
     */
    bool has_node(int node);

    /**
     * \brief Get all nodes in the graph.
     * \return The set of all nodes in the graph.
     */
    std::unordered_set<int> get_all_nodes();

    /**
     * \brief Get all adjacent nodes to a node.
     * \param node The node to get adjacent nodes to.
     * \return The set of adjacent nodes to the given node.
     */
    std::unordered_set<int> get_adjacent_nodes(int node);

    /**
     * \brief Get the shortest path between two nodes.
     * \param start The start node.
     * \param goal The goal node.
     * \return The shortest path between the two nodes.
     */
    std::vector<int> get_shortest_path(int start, int goal);
};

#endif // POSE_GRAPH_HPP_
