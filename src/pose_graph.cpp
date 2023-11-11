// Copyright (c) 2023 Kosuke Suzuki
// Released under the MIT license
// https://opensource.org/licenses/mit-license.php

#include <pose_graph/pose_graph.hpp>

#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <queue>
#include <algorithm>


void PoseGraph::add_edge(int a, int b)
{
    // Avoid duplicate edges
    if (adjacency_list[a].find(b) != adjacency_list[a].end())
        return;
    // Avoid self loops
    if (a == b)
        return;
    // Add edge in both directions
    adjacency_list[a].insert(b);
    adjacency_list[b].insert(a);
}

void PoseGraph::remove_node(int a)
{
    // Remove all edges to a
    if (adjacency_list.find(a) != adjacency_list.end())
    {
        for (int neighbor : adjacency_list[a]) 
        {
            adjacency_list[neighbor].erase(a);
        }
        adjacency_list.erase(a);
    }
}

bool PoseGraph::has_edge(int a, int b)
{
    return adjacency_list[a].find(b) != adjacency_list[a].end();
}

bool PoseGraph::has_node(int node)
{
    return adjacency_list.find(node) != adjacency_list.end();
}

std::unordered_set<int> PoseGraph::get_all_nodes()
{
    std::unordered_set<int> nodes;
    for (const auto& node_entry : adjacency_list)
    {
        nodes.insert(node_entry.first);
    }
    return nodes;
}

std::unordered_set<int> PoseGraph::get_adjacent_nodes(int node)
{
    std::unordered_set<int> adjacent_nodes
    ;
    if (adjacency_list.find(node) != adjacency_list.end())
    {
        for (int neighbor : adjacency_list[node])
        {
            adjacent_nodes.insert(neighbor);
        }
    }
    return adjacent_nodes;
}

std::vector<int> PoseGraph::get_shortest_path(int start, int end)
{
    std::unordered_map<int, int> parent;
    std::queue<int> q;
    q.push(start);
    parent[start] = -1;

    while (!q.empty())
    {
        int current = q.front();
        q.pop();

        if (current == end)
        {
            break;
        }

        for (int neighbor : adjacency_list[current])
        {
            if (parent.find(neighbor) == parent.end())
            {
                q.push(neighbor);
                parent[neighbor] = current;
            }
        }
    }

    std::vector<int> path;
    if (parent.find(end) != parent.end())
    {
        int current = end;
        while (current != -1)
        {
            path.push_back(current);
            current = parent[current];
        }
        std::reverse(path.begin(), path.end());
    }

    return path;
}
