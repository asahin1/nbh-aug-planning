#include "myNode2D.hpp"

myNode2D *myNode2D::oneStepRollback()
{
    // This is an attempt (for SStar, for AStar it is supposed to work as intended)
    // to rollback one step
#ifdef DOSL_ALGORITHM_SStar
    myNode2D *cfv{nullptr};
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

void myNode2D::findNeighborhoodCenter()
{
    myNode2D *recursive_ptr{this};
    for (int i{0}; i < R_ROLLBACK_RADIUS; i++)
    {
        recursive_ptr = recursive_ptr->oneStepRollback();
    }
    neighborhoodCenterNode = recursive_ptr;
}

bool myNode2D::isCoordsEqual(const myNode2D &n) const
{ // Coordinate based comparison
    return ((x == n.x) && (y == n.y));
}

bool myNode2D::hasNeighborhoodIntersection(myNode2D &n)
{
    // Collect predecessors
    if (neighborhood.empty())
        getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_NEIGHBORHOOD_SEARCH_DEPTH);
    if (n.neighborhood.empty())
        n.getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_NEIGHBORHOOD_SEARCH_DEPTH);
    // Check for overlap
    int commonNeighbors = 0;
    std::unordered_set<myNode2D *> commonNodes;
    for (auto &i : neighborhood)
    {
        if (i->isCutPoint)
            continue;
        if (n.neighborhood.count(i) != 0)
        {
            commonNeighbors++;
            commonNodes.insert(i);
        }
    }
    int overlapDenominator = std::max(neighborhood.size(), n.neighborhood.size());
    double neighborhoodOverlap = static_cast<double>(commonNeighbors) * 100 / overlapDenominator;

    if (neighborhoodOverlap > 0)
    {
        if (isCutPoint)
        {
            n.isCutPoint = true;
            return true;
        }
        if (neighborhoodOverlap <= R_OVERLAP_FOR_CP)
        {
            if (R_CUT_POINT_CHECK)
                cutPointCheck(n);
        }
        return true;
    }

    return false;
}

double myNode2D::getDistance(const myNode2D &n) const
{
    double dx = x - n.x;
    double dy = y - n.y;
    return sqrt(dx * dx + dy * dy);
}

// Comparison
bool myNode2D::operator==(myNode2D &n)
{
    if (!isCoordsEqual(n)) // corrdinate based comparison (this is necessary)
        return (false);

    if (isCutPoint || n.isCutPoint)
    {
        genNo = std::min(genNo, n.genNo);
        return true;
    }

    if (!hasNeighborhoodIntersection(n))
    {
        return (false);
    }

    genNo = std::min(genNo, n.genNo);
    return (true);
}

// Constructors
myNode2D::myNode2D(COORD_TYPE xx, COORD_TYPE yy) : x(xx), y(yy) {}

// Inherited functions being overwritten
int myNode2D::getHashBin(void) const
{
    return (100 * abs(x) + abs(y)); // SB: better hash function
}

void myNode2D::print(std::string head, std::string tail) const
{ // Print this node
    _dosl_cout << std::setw(2) << std::right << head
               << std::setw(4) << std::right << "[x=" << x
               << ", y=" << y << "]"
               << std::setfill(' ')
               << _dosl_endl;
}

myNode2D myNode2D::operator+(const myNode2D &b) const
{ // n1 + n2
    myNode2D ret = (*this);
    ret.x += b.x;
    ret.y += b.y;
    return (ret);
}

myNode2D myNode2D::operator*(const double &c) const
{ // n1 * c (right scalar multiplication)
    myNode2D ret = (*this);
    ret.x *= c;
    ret.y *= c;
    return (ret);
}