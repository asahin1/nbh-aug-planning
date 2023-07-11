#include "dosl/planners/AStar_original.tcc"
#include "dosl/planners/SStar_original.tcc"
#include "dosl/planners/ThetaStar_original.tcc"

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

#include <dosl/aux-utils/cvParseMap2d.hpp>

#define SEARCH2_ALG AStar_original

// #define HEURISTIC_WEIGHT 0.8

template <class nodeType>
class cutPointSearchNode : public SEARCH2_ALG::Node<cutPointSearchNode<nodeType>, double>
{
public:
    nodeType *np; // required member of nodeType: unordered_map<nodeType*,double> successors;
    // bool came_from_pinch_point; // *SB: allow neighborhood search to propagate through pinch points, but mark them.
    // bool isCutPoint = false;
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
        // return (np == n.np);
    }

    // An efficint hash function, 'getHashBin', for node type is desired, but is optional.
    int getHashBin(void) { return ((int)((long int)np % 1000)); }
};

template <class nodeType>
class cutPointSearchProblem : public SEARCH2_ALG::Algorithm<cutPointSearchProblem<nodeType>, cutPointSearchNode<nodeType>, double>
{
public:
    using SEARCH2_ALG::Algorithm<cutPointSearchProblem<nodeType>, cutPointSearchNode<nodeType>, double>::all_nodes_set_p;

    typedef cutPointSearchNode<nodeType> CPSN;
    CPSN start;
    CPSN goal;
    int min_generations;
    double outerRadius;
    CPSN primary_search_start;
    bool reachedGoal{false};
    double pathLength{-1};
    cvParseMap2d my_map;
    cv::Mat occupancyMap;
    // Alternatively
    std::unordered_set<nodeType *> neighborhood;
    cutPointSearchProblem(nodeType *p, double o_r, int mg, nodeType *g, nodeType *s, cv::Mat primary_search_display)
        : outerRadius(o_r), min_generations(mg)
    {
        start = CPSN(p);
        goal = CPSN(g);
        primary_search_start = CPSN(s);
    }
    cutPointSearchProblem(nodeType *p, double o_r, int mg, nodeType *g, nodeType *s, cv::Mat primary_search_display, cvParseMap2d search_map)
        : outerRadius(o_r), min_generations(mg), my_map(search_map)
    {
        start = CPSN(p);
        goal = CPSN(g);
        primary_search_start = CPSN(s);
    }
    cutPointSearchProblem(nodeType *p, double o_r, int mg, nodeType *g, nodeType *s, cv::Mat primary_search_display, cv::Mat occupancy_map)
        : outerRadius(o_r), min_generations(mg), occupancyMap(occupancy_map)
    {
        start = CPSN(p);
        goal = CPSN(g);
        primary_search_start = CPSN(s);
    }

    // void getSuccessors(NSN &n, std::vector<NSN> *s, std::vector<int> *c)
    void getSuccessors(CPSN &n, std::vector<CPSN> *s, std::vector<double> *c)
    {
        // This function should account for obstacles, constraints and size of environment
        for (auto it = n.np->successors.begin(); it != n.np->successors.end(); ++it)
        {
            if (!it->first->successors_created)
                continue;
            if (it->first->isCutPoint)
                continue;
            // if (it->first->g_score >= n.np->g_score)
            //     continue;
            CPSN tn(it->first);
            tn.generation_no = n.generation_no + 1;
            s->push_back(tn);
            c->push_back(it->second);
            // Use euclidean distance without the cost multiplier
            // double cost = n.np->getDist(*it->first);
            // c->push_back(cost);
        }
    }

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
            if (occupancyMap.at<int>((int)round(xx), (int)round(yy), (int)round(zz)))
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
            if (my_map.isObstacle((int)round(xx), (int)round(yy)))
                return (false);
        }
        return (true);
#endif
    }

    double getHeuristics(CPSN &n)
    {
        // std::cout << n.np->g_score << std::endl;
        double dx = start.np->x - goal.np->x;
        double dy = start.np->y - goal.np->y;
        return sqrt(dx * dx + dy * dy);
        // return 0;
        // double dx = primary_search_start.np->x - n.np->x;
        // double dy = primary_search_start.np->y - n.np->y;
        // return 0.5*(sqrt(dx * dx + dy * dy));
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
        // if (n.np->isCoordsEqual(*primary_search_start.np))
        // {
        //     // _dosl_cout << "Secondary search: Arrived primary start" << _dosl_endl;
        //     return true;
        // }
        // if (n.generation_no < min_generations)
        //     return false;
        if (n == goal)
        {
            reachedGoal = true;
            pathLength = n.g_score;
            return true;
        }
        if (n.g_score > outerRadius)
        {
            reachedGoal = false;
            return true;
        }
        return false;
    }

    std::vector<nodeType *> getPath()
    {
        std::vector<nodeType *> path;
        if (reachedGoal)
        {
            auto MPS_path = this->reconstruct_pointer_path(goal);
            for (auto i : MPS_path)
                path.push_back(i->np);
        }
        return path;
    }
};
