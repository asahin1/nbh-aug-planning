// Header file for the myNode class, for now class implementations are included
// Guards
#ifndef _MYNODE3D_H
#define _MYNODE3D_H

// Standard libraries
// #include <unordered_set>

// DOSL macros
#define _DOSL_CHECK_GRAPH_DIRECTIONALITY 2 // set 2 for auto-fix
#ifndef _DOSL_ALGORITHM                    // can pass at command line during compilation: -D_DOSL_ALGORITHM=AStar
#define _DOSL_ALGORITHM SStar
#endif
// DOSL
#include <dosl/dosl>

// User made
#include "neighborhoodSearchProblem.cpp" // coded by SB
#include "cutPointSearchProblem.cpp"
#include "tools.hpp" // some global variables and helper functions

// Declaration of myNode class
class myNode3D : public _DOSL_ALGORITHM::Node<myNode3D, double>
{
public:
    COORD_TYPE x, y, z;                          // coordinates
    myNode3D *parent = nullptr;                  // pointer to parent (at generation time)
    myNode3D *neighborhoodCenterNode = nullptr;  // pointer to furthest grandparent (updated during predecessor rollback)
    std::unordered_set<myNode3D *> neighborhood; // set of predecessors (generated during comparison)
    bool isCutPoint = false;                     // merge point flag
    int genNo{0};

    // Updates and checks
    myNode3D *oneStepRollback();
    void findNeighborhoodCenter();
    void getNeighborhood(double nbRadius, int minSearchDepth);
    void cutPointCheck(myNode3D &n);
    void generateCutPointRegion();

    bool isCoordsEqual(const myNode3D &n) const;   // coordinate comparison
    bool hasNeighborhoodIntersection(myNode3D &n); // predecessor comparison

    double getDistance(const myNode3D &n) const;

    // *** This must be defined for the node
    // Comparison
    bool operator==(myNode3D &n);

    // constructors
    myNode3D() {}
    myNode3D(COORD_TYPE xx, COORD_TYPE yy, COORD_TYPE zz);
    // myNode (const myNode &source); // Copy constructor

    // Inherited functions being overwritten
    int getHashBin(void) const;

    void print(std::string head = "", std::string tail = "") const;

    // ---------------------------------------------------------------
    // Operator overloading for convex combination -- for Path Recnstruction in SStar algorithm

    myNode3D operator+(const myNode3D &b) const;

    myNode3D operator*(const double &c) const;
};

#endif