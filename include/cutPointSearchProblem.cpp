// Node and search problem definitions for cut point search
// Cut point search is being used to check the distance between path points p_w and p_v'
// Refer to Algorithm 4 Line 8 (searchOnGraph)

#include "dosl/planners/AStar_original.tcc"
#include "dosl/planners/SStar_original.tcc"
#include "dosl/planners/ThetaStar_original.tcc"

#include <dosl/aux-utils/cvParseMap2d.hpp>

// Can potentially use other algorithms for cut point search (AStar works the best for now)
#define SEARCH2_ALG AStar_original

template <class nodeType>
class cutPointSearchNode : public SEARCH2_ALG::Node<cutPointSearchNode<nodeType>, double>
{
public:
    nodeType *np;
    int generation_no{0};
    cutPointSearchNode() {}
    cutPointSearchNode(nodeType *p) : np(p) {}

    // The comparison operator must be defined for the node type.
    bool operator==(cutPointSearchNode<nodeType> &n)
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
class cutPointSearchProblem : public SEARCH2_ALG::Algorithm<cutPointSearchProblem<nodeType>, cutPointSearchNode<nodeType>, double>
{
public:
    // Existing nodes on the graph
    using SEARCH2_ALG::Algorithm<cutPointSearchProblem<nodeType>, cutPointSearchNode<nodeType>, double>::all_nodes_set_p;
    typedef cutPointSearchNode<nodeType> CPSN;

    CPSN start;
    CPSN goal;
    double maxSearchRadius;
    CPSN primary_search_start;
    bool reachedGoal{false};
    double pathLength{-1};
    cvParseMap2d env2D;
    cv::Mat env3D;

    cutPointSearchProblem(nodeType *p, double o_r, nodeType *g, nodeType *s)
        : maxSearchRadius(o_r)
    {
        start = CPSN(p);
        goal = CPSN(g);
        primary_search_start = CPSN(s);
    }
    cutPointSearchProblem(nodeType *p, double o_r, nodeType *g, nodeType *s, cvParseMap2d search_map_2D)
        : maxSearchRadius(o_r), env2D(search_map_2D)
    {
        start = CPSN(p);
        goal = CPSN(g);
        primary_search_start = CPSN(s);
    }
    cutPointSearchProblem(nodeType *p, double o_r, nodeType *g, nodeType *s, cv::Mat search_map_3D)
        : maxSearchRadius(o_r), env3D(search_map_3D)
    {
        start = CPSN(p);
        goal = CPSN(g);
        primary_search_start = CPSN(s);
    }

    void getSuccessors(CPSN &n, std::vector<CPSN> *s, std::vector<double> *c)
    {
        // Follow through the existing edges in the graph
        for (auto it = n.np->successors.begin(); it != n.np->successors.end(); ++it)
        {
            // Skip any nodes whose successors are not created or that are cut points
            if (!it->first->successors_created)
                continue;
            if (it->first->isCutPoint)
                continue;
            CPSN tn(it->first);
            tn.generation_no = n.generation_no + 1;
            s->push_back(tn);
            c->push_back(it->second);
        }
    }

    // Visibility check (required for ThetaStar only)
    bool isSegmentFree(CPSN &n1, CPSN &n2, double *c)
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

    double getHeuristics(CPSN &n)
    {
        // Euclidean distance heuristic
        double dx = start.np->x - goal.np->x;
        double dy = start.np->y - goal.np->y;
        return sqrt(dx * dx + dy * dy);
        // return 0;
    }

    std::vector<CPSN> getStartNodes(void)
    {
        std::vector<CPSN> startNodes;
        startNodes.push_back(start);
        return (startNodes);
    }

    bool stopSearch(CPSN &n)
    {
        if (n == goal)                   // Goal reached within radius
        {                                //
            reachedGoal = true;          //
            pathLength = n.g_score;      //
            return true;                 //
        }                                //
        if (n.g_score > maxSearchRadius) // Maximum radius exceeded
        {
            reachedGoal = false;
            return true;
        }
        return false;
    }
};
