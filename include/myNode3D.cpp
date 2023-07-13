#include "myNode3D.hpp"

myNode3D *myNode3D::oneStepRollback()
{
#ifdef DOSL_ALGORITHM_SStar
    myNode3D *cfv{nullptr};
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

void myNode3D::findNeighborhoodCenter()
{
    myNode3D *recursive_ptr{this};
    for (int i{0}; i < R_ROLLBACK_RADIUS; i++)
    {
        recursive_ptr = recursive_ptr->oneStepRollback();
    }
    neighborhoodCenterNode = recursive_ptr;
}

bool myNode3D::isCoordsEqual(const myNode3D &n) const
{
    return ((x == n.x) && (y == n.y) && (z == n.z));
}

bool myNode3D::hasNeighborhoodIntersection(myNode3D &n)
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
        if (i->isCutPoint)
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
            if (neighborhoodOverlap < R_OVERLAP_FOR_CP)
                cutPointCheck(n);
        }
        return true;
    }

    return false;
}

double myNode3D::getDistance(const myNode3D &n) const
{
    double dx = x - n.x;
    double dy = y - n.y;
    double dz = z - n.z;
    return sqrt(dx * dx + dy * dy + dz * dz);
}

// Comparison
bool myNode3D::operator==(myNode3D &n)
{
    if (!isCoordsEqual(n))
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

myNode3D::myNode3D(COORD_TYPE xx, COORD_TYPE yy, COORD_TYPE zz) : x(xx), y(yy), z(zz) {}

int myNode3D::getHashBin(void) const
{
    return (100 * abs(x) + abs(y) + 10 * abs(z));
}

void myNode3D::print(std::string head, std::string tail) const
{
    _dosl_cout << std::setw(2) << std::right << head
               << std::setw(4) << std::right << "[x=" << x
               << ", y=" << y << ", z=" << z << "]"
               << std::setfill(' ')
               << _dosl_endl;
}

myNode3D myNode3D::operator+(const myNode3D &b) const
{ // n1 + n2
    myNode3D ret = (*this);
    ret.x += b.x;
    ret.y += b.y;
    ret.z += b.z;
    return (ret);
}

myNode3D myNode3D::operator*(const double &c) const
{ // n1 * c (right scalar multiplication)
    myNode3D ret = (*this);
    ret.x *= c;
    ret.y *= c;
    ret.z *= c;
    return (ret);
}