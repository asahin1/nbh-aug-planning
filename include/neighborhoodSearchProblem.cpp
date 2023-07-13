// Node and search problem class definitions for neighborhood search
// Refer to Algorithm 3 (computePNS)

#include "dosl/planners/AStar_original.tcc"
#include "dosl/planners/SStar_original.tcc"
#include "dosl/planners/ThetaStar_original.tcc"

#include <dosl/aux-utils/cvParseMap2d.hpp>

// Can potentially use other algorithms for neighborhood search (AStar works the best for now)
#define SEARCH2_ALG AStar_original

template <class nodeType>
class neighborhoodSearchNode : public SEARCH2_ALG::Node<neighborhoodSearchNode<nodeType>, double>
{
public:
    nodeType *np;
    int generation_no{0};
    neighborhoodSearchNode() {}
    neighborhoodSearchNode(nodeType *p) : np(p) {}

    // The comparison operator must be defined for the node type.
    bool operator==(neighborhoodSearchNode<nodeType> &n)
    {
        if (np == n.np)
        {
            // For more consistent generation numbers
            int min_gen_no{std::min(generation_no, n.generation_no)};
            generation_no = min_gen_no;
            n.generation_no = min_gen_no;
            return true;
        }
        return false;
    }

    int getHashBin(void) { return ((int)((long int)np % 1000)); }
};

template <class nodeType>
class neighborhoodSearchProblem : public SEARCH2_ALG::Algorithm<neighborhoodSearchProblem<nodeType>, neighborhoodSearchNode<nodeType>, double>
{
public:
    // Existing nodes on the graph
    using SEARCH2_ALG::Algorithm<neighborhoodSearchProblem<nodeType>, neighborhoodSearchNode<nodeType>, double>::all_nodes_set_p;
    typedef neighborhoodSearchNode<nodeType> NSN;

    NSN centerNode;
    int minSearchDepth;
    double nb_radius;
    double heuristic_weight;
    NSN primary_search_start;
    cvParseMap2d env2D;
    cv::Mat env3D;
    std::unordered_set<nodeType *> neighborhood;

    neighborhoodSearchProblem(nodeType *p, double nr, double hw, int mg, nodeType *s)
        : nb_radius(nr), heuristic_weight(hw), minSearchDepth(mg)
    {
        centerNode = NSN(p);
        primary_search_start = NSN(s);
    }
    neighborhoodSearchProblem(nodeType *p, double nr, double hw, int mg, nodeType *s, cvParseMap2d &search_map_2D)
        : nb_radius(nr), heuristic_weight(hw), minSearchDepth(mg), env2D(search_map_2D)
    {
        centerNode = NSN(p);
        primary_search_start = NSN(s);
    }
    neighborhoodSearchProblem(nodeType *p, double nr, double hw, int mg, nodeType *s, cv::Mat &search_map_3D)
        : nb_radius(nr), heuristic_weight(hw), minSearchDepth(mg), env3D(search_map_3D)
    {
        centerNode = NSN(p);
        primary_search_start = NSN(s);
    }

    void getSuccessors(NSN &n, std::vector<NSN> *s, std::vector<double> *c)
    {
        // Follow through the existing edges in the graph
        for (auto it = n.np->successors.begin(); it != n.np->successors.end(); ++it)
        {
            // Skip any nodes whose successors are not created or that are cut points
            if (!it->first->successors_created)
                continue;
            if (it->first->isCutPoint)
                continue;
            NSN tn(it->first);
            tn.generation_no = n.generation_no + 1;
            s->push_back(tn);
            c->push_back(it->second);
        }
    }

    // Visibility check (required for ThetaStar only)
    bool isSegmentFree(NSN &n1, NSN &n2, double *c)
    {
#ifdef _SEARCHPROBLEM3D_H
        double dx = (double)(n2.np->x - n1.np->x), dy = (double)(n2.np->y - n1.np->y), dz = (double)(n2.np->z - n1.np->z);
        *c = sqrt(dx * dx + dy * dy + dz * dz);

        double step_count = 2.0 * ceil(*c);
        double xstep = dx / step_count, ystep = dy / step_count, zstep = dz / step_count;

        double xx, yy, zz;
        for (int a = 0; a < step_count; ++a)
        {
            xx = n1.np->x + a * xstep;
            yy = n1.np->y + a * ystep;
            zz = n1.np->z + a * zstep;
            if (env3D.at<int>((int)round(xx), (int)round(yy), (int)round(zz)))
                return (false);
        }
        return (true);
#else
        double dx = (double)(n2.np->x - n1.np->x), dy = (double)(n2.np->y - n1.np->y);
        *c = sqrt(dx * dx + dy * dy);

        double step_count = 2.0 * ceil(*c);
        double xstep = dx / step_count, ystep = dy / step_count;

        double xx, yy;
        for (int a = 0; a < step_count; ++a)
        {
            xx = n1.np->x + a * xstep;
            yy = n1.np->y + a * ystep;
            if (env2D.isObstacle((int)round(xx), (int)round(yy)))
                return (false);
        }
        return (true);
#endif
    }

    double getHeuristics(NSN &n)
    {
        return heuristic_weight * (n.np->g_score);
        // return 0;
    }

    std::vector<NSN> getStartNodes(void)
    {
        std::vector<NSN> startNodes;
        startNodes.push_back(centerNode);
        return (startNodes);
    }

    bool stopSearch(NSN &n)
    {
        if (n.np->isCoordsEqual(*primary_search_start.np)) // No need to go further if the start node of the primary search
            return true;                                   // is reached (that will be common for all)
        if (n.generation_no < minSearchDepth)              // minimum depth check
            return false;                                  //
        if (n.g_score > nb_radius)                         // neighborhood radius check
            return true;
        return false;
    }

    // Collect all expanded nodes
    std::unordered_set<nodeType *> collectNeighborhood()
    {

        for (int b = 0; b < all_nodes_set_p->hash_table_size; ++b)
            for (int c = 0; c < all_nodes_set_p->HashTable[b].size(); ++c)
            {
                NSN *dummy = all_nodes_set_p->HashTable[b][c];
                if (dummy->expanded)
                    neighborhood.insert(dummy->np);
            }
        return neighborhood;
    }
};
