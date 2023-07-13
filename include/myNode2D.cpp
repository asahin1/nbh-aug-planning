#include "myNode2D.hpp"

myNode2D *myNode2D::oneStepRollback()
{
#ifdef DOSL_ALGORITHM_SStar
    myNode2D *cfv{nullptr};
    // Follow through the node with highest weight in the simplex
    if (CameFromSimplex != NULL)
    {
        double maxWeight{-1};
        for (auto &s : CameFromSimplex->p)
        {
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
    // Follow through came from nodes
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
{
    return ((x == n.x) && (y == n.y));
}

bool myNode2D::hasNeighborhoodIntersection(myNode2D &n)
{
    // Generate neighborhood (if not generated before)
    if (neighborhood.empty())
        getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_NEIGHBORHOOD_SEARCH_DEPTH);
    if (n.neighborhood.empty())
        n.getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_NEIGHBORHOOD_SEARCH_DEPTH);

    // Check for intersection
    int commonNeighbors = 0;
    for (auto &i : neighborhood)
    {
        if (i->isCutPoint) // Cut points do not count
            continue;
        if (n.neighborhood.count(i) != 0)
            commonNeighbors++;
    }

    int overlapDenominator = std::max(neighborhood.size(), n.neighborhood.size());
    double neighborhoodOverlap = static_cast<double>(commonNeighbors) * 100 / overlapDenominator;

    if (neighborhoodOverlap > 0)
    {
        if (R_CUT_POINT_CHECK)
        {
            if (isCutPoint)
            {
                n.isCutPoint = true; // Nodes were going to be identified as the same anyway, no need to check again
                return true;
            }
            if (neighborhoodOverlap <= R_OVERLAP_FOR_CP)
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
        return (false);

    genNo = std::min(genNo, n.genNo);
    return (true);
}

myNode2D::myNode2D(COORD_TYPE xx, COORD_TYPE yy) : x(xx), y(yy) {}

int myNode2D::getHashBin(void) const
{
    return (100 * abs(x) + abs(y));
}

void myNode2D::print(std::string head, std::string tail) const
{
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