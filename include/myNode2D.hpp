// Header file for the myNode2D class

// Guards
#ifndef _MYNODE2D_H
#define _MYNODE2D_H

// DOSL
#define _DOSL_CHECK_GRAPH_DIRECTIONALITY 2 // set 2 for auto-fix
#ifndef _DOSL_ALGORITHM                    // can pass at command line during compilation: -D_DOSL_ALGORITHM=AStar
#define _DOSL_ALGORITHM SStar
#endif
#include <dosl/dosl>

// Planning related
#include "neighborhoodSearchProblem.cpp"
#include "cutPointSearchProblem.cpp"
#include "tools.hpp"

class myNode2D : public _DOSL_ALGORITHM::Node<myNode2D, double>
{
public:
    COORD_TYPE x, y;                             // coordinates
    myNode2D *parent = nullptr;                  // parent node
    myNode2D *neighborhoodCenterNode = nullptr;  // center of the neighborhood (obtained by rollback)
    std::unordered_set<myNode2D *> neighborhood; // path neighborhood set
    bool isCutPoint = false;                     // cut point flag
    int genNo{0};                                // search depth maintained as generation number

    // Updates and checks
    myNode2D *oneStepRollback();
    void findNeighborhoodCenter();
    void getNeighborhood(double nbRadius, int minSearchDepth);
    void cutPointCheck(myNode2D &n);
    void generateCutPointRegion();

    double getDistance(const myNode2D &n) const;

    bool isCoordsEqual(const myNode2D &n) const;   // coordinate comparison
    bool hasNeighborhoodIntersection(myNode2D &n); // neighborhood comparison
    // This must be defined for the node
    bool operator==(myNode2D &n); // comparison

    // constructors
    myNode2D() {}
    myNode2D(COORD_TYPE xx, COORD_TYPE yy);

    // Inherited functions being overwritten
    int getHashBin(void) const;
    void print(std::string head = "", std::string tail = "") const;

    // ---------------------------------------------------------------
    // Operator overloading for convex combination -- for Path Reconstruction in SStar algorithm
    myNode2D operator+(const myNode2D &b) const;
    myNode2D operator*(const double &c) const;
};

#endif