#include "3d_myNode.hpp"

void myNode::printNeighborhood() const
{ // Print all predecessors
    print("Printing predecessors of: ");
    int idx = 1;
    for (auto &p : neighborhood)
    {
        p->print(std::to_string(idx) + ". ");
        idx++;
    }
}

void myNode::printSuccessors() const
{ // Print all successors
    print("Printing successors of: ");
    int idx = 1;
    for (auto &s : successors)
    {
        s.first->print(std::to_string(idx) + ".(cost: " + std::to_string(s.second) + ") ");
        idx++;
    }
}

void myNode::printParents() const
{ // Print all parents
    print("Printing parents of: ");
    int idx = 1;
    for (auto &p : parent_list)
    {
        p.first->print(std::to_string(idx) + ". ");
        idx++;
    }
}

void myNode::printCameFromSimplex() const
{

#ifdef DOSL_ALGORITHM_SStar
    print("Printing CameFromSimplex of: ");
    int idx = 1;
    for (const auto &c : CameFromSimplex->p)
    {
        c->print(std::to_string(idx) + ". ");
        _dosl_cout << "Weight of simpex " << idx << ": " << CameFromSimplex->w[c] << _dosl_endl;
        idx++;
    }
    _dosl_cout << "Printing successors for cost check: " << _dosl_endl;
    printSuccessors();
#endif
}

myNode *myNode::oneStepRollback()
{
    // This is an attempt (for SStar, for AStar it is supposed to work as intended)
    // to rollback one step
#ifdef DOSL_ALGORITHM_SStar
    myNode *cfv{nullptr};
    if (CameFromSimplex != NULL)
    {
        double maxWeight{-1};
        for (auto &s : CameFromSimplex->p)
        {
            // if (CameFromSimplex->w[s] > maxWeight && !s->isMergePoint) // this causes segmentation faults (occasionally)
            if (CameFromSimplex->w[s] > maxWeight)
            {
                maxWeight = CameFromSimplex->w[s];
                cfv = s;
            }
        }
    }
    if (cfv)
        return cfv;
#else
    if (came_from)
        return came_from;
#endif
    else
        return parent;
}

void myNode::findFurthestGrandParent()
{
    myNode *recursive_ptr{this};
    for (int i{0}; i < R_ROLLBACK; i++)
    {
        recursive_ptr = recursive_ptr->oneStepRollback();
    }
    furthest_grandparent = recursive_ptr;
}

void myNode::mergePointCheck(myNode &n)
{
    getMergePointSets(R_MP_LOWER_THRESHOLD, R_MP_UPPER_THRESHOLD, R_MP_UPPER_THRESHOLD);
    n.getMergePointSets(R_MP_LOWER_THRESHOLD, R_MP_UPPER_THRESHOLD, R_MP_UPPER_THRESHOLD);
    // Should not have any common vertices within the smaller neighborhood (otherwise on the same branch)
    for (auto it1 = mp_lower_nb.begin(); it1 != mp_lower_nb.end(); ++it1)
    {
        for (auto it2 = n.mp_lower_nb.begin(); it2 != n.mp_lower_nb.end(); ++it2)
        {
            if (*it1 == *it2)
            {
                // (*it1)->print("it1: ");
                // (*it2)->print("it2: ");
                return;
            }
        }
    }
    // Should have common vertices within the larger neighborhood (otherwise there is an obstacle)
    for (auto it1 = mp_upper_nb.begin(); it1 != mp_upper_nb.end(); ++it1)
    {
        for (auto it2 = n.mp_upper_nb.begin(); it2 != n.mp_upper_nb.end(); ++it2)
        {
            if (*it1 == *it2)
            {
                // (*it1)->print("it1: ");
                // (*it2)->print("it2: ");
                isMergePoint = true;
                n.isMergePoint = true;
                plotMergePoint();
                n.plotMergePoint();
                if (R_PLOT_MP_NB)
                    plotMPNBs(&n);
                return;
            }
        }
    }
    return;
}

bool myNode::isCoordsEqual(const myNode &n) const
{ // Coordinate based comparison
    return ((x == n.x) && (y == n.y) && (z == n.z));
}

bool myNode::hasNeighborhoodIntersection(myNode &n)
{
    // Collect predecessors
    if (neighborhood.empty())
        getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_MIN_NB_GENERATIONS);
    if (n.neighborhood.empty())
        n.getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_MIN_NB_GENERATIONS);
    // Check for overlap
    int commonNeighbors = 0;
    std::unordered_set<myNode *> commonNodes;
    for (auto &i : neighborhood)
    {
        if (i->isMergePoint)
            continue;
        if (n.neighborhood.count(i) != 0)
        {
            commonNeighbors++;
            commonNodes.insert(i);
        }
    }
    int overlapDenominator = std::max(neighborhood.size(), n.neighborhood.size());
    double neighborhoodOverlap = static_cast<double>(commonNeighbors) * 100 / overlapDenominator;

    if (std::min(neighborhood.size(), n.neighborhood.size()) < R_NEIGHBORHOOD_SIZE_THRESHOLD)
    {
        if (neighborhoodOverlap > 0)
        {
            // if (R_PLOT_NEIGHBORHOOD && y == 50 && x > 65)
            // {
            //     plotNeighborhoods(&n, commonNodes);
            // }

            if (isMergePoint)
            {
                n.isMergePoint = true;
                return true;
            }
            if (neighborhoodOverlap < R_OVERLAP_FOR_MP)
            {
                if (R_MP_CHECK)
                    alternativeMergePointCheck(n);
            }
            return true;
        }
    }
    else
    {
        if (neighborhoodOverlap > R_NEIGHBORHOOD_OVERLAP_THRESHOLD)
        {
            // if (R_PLOT_NEIGHBORHOOD && y == 50 && x > 65)
            // {
            //     plotNeighborhoods(&n, commonNodes);
            // }
            if (isMergePoint)
            {
                n.isMergePoint = true;
                return true;
            }
            if (neighborhoodOverlap < R_OVERLAP_FOR_MP)
            {
                if (R_MP_CHECK)
                    alternativeMergePointCheck(n);
            }
            return true;
        }
    }
    if (R_PLOT_NEIGHBORHOOD)
    {
        plotNeighborhoods(&n, commonNodes);
    }
    return false;
}

void myNode::mergeNodeElements(myNode &n)
{
    mergeUnordered(this->parent_list, n.parent_list); // merge parent lists
}

double myNode::getDist(const myNode &n) const
{
    double dx = x - n.x;
    double dy = y - n.y;
    double dz = z - n.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Comparison
bool myNode::operator==(myNode &n)
{
    if (!isCoordsEqual(n)) // corrdinate based comparison (this is necessary)
        return (false);

#ifdef DOSL_ALGORITHM_SStar
    // if (y = 50 && x > 50)
    if (genNo >= R_LINEAGE_DATA_GENERATION_THRESHOLD && n.genNo >= R_LINEAGE_DATA_GENERATION_THRESHOLD)
    // if (lineage_data.generation > R_LINEAGE_DATA_GENERATION_THRESHOLD || n.lineage_data.generation > R_LINEAGE_DATA_GENERATION_THRESHOLD)
    { // *SB: don't do this near the start
#endif
        // mergePointCheck(n);
        // if (abs(y - 50) < 2 && x > 65)
        // {
        //     rollbackWithPlot();
        //     // if (R_MP_CHECK)
        //     //     alternativeMergePointCheck(n);
        // }

        if (isMergePoint || n.isMergePoint)
        {
            mergeNodeElements(n); // if true, merge elements
            genNo = std::min(genNo, n.genNo);
            return true;
        }

        if (!hasNeighborhoodIntersection(n))
        {
            return (false);
        }
        mergeNodeElements(n); // if true, merge elements
#ifdef DOSL_ALGORITHM_SStar
    }
#endif
    genNo = std::min(genNo, n.genNo);
    return (true);
}

// Constructors
myNode::myNode(COORD_TYPE xx, COORD_TYPE yy, COORD_TYPE zz) : x(xx), y(yy), z(zz) {}

// Inherited functions being overwritten
int myNode::getHashBin(void) const
{
    return (100 * abs(x) + abs(y) + 10 * abs(z)); // SB: better hash function
}

void myNode::print(std::string head, std::string tail) const
{ // Print this node
    _dosl_cout << std::setw(60) << std::right << head
               << std::setw(4) << std::right << "[x=" << x
               << ", y=" << y << ", z=" << z << "]"
               << std::setfill('-')
               << std::setw(20) << std::right << "Address: " << this
               << std::setfill(' ')
               << _dosl_endl;
}

myNode myNode::operator+(const myNode &b) const
{ // n1 + n2
    myNode ret = (*this);
    ret.x += b.x;
    ret.y += b.y;
    ret.z += b.z;
    return (ret);
}

myNode myNode::operator*(const double &c) const
{ // n1 * c (right scalar multiplication)
    myNode ret = (*this);
    ret.x *= c;
    ret.y *= c;
    ret.z *= c;
    return (ret);
}