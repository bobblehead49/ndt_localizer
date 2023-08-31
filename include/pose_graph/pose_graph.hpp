# ifndef POSE_GRAPH_HPP_
# define POSE_GRAPH_HPP_

#include <unordered_map>
#include <unordered_set>
#include <vector>


class PoseGraph
{
private:
    std::unordered_map<int, std::unordered_set<int>> adjacency_list;

public:
    PoseGraph() {}
    ~PoseGraph() {}

    void add_edge(int a, int b);
    void remove_node(int a);
    bool has_edge(int a, int b);
    bool has_node(int node);
    std::unordered_set<int> get_all_nodes();
    std::unordered_set<int> get_adjacent_nodes(int node);
    std::vector<int> get_shortest_path(int start, int goal);
};

#endif // POSE_GRAPH_HPP_
