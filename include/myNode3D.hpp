// Header file for the myNode3D class

// Guards
#ifndef _MYNODE3D_H
#define _MYNODE3D_H

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

class myNode3D : public _DOSL_ALGORITHM::Node<myNode3D, double>
{
public:
    COORD_TYPE x, y, z;                          // coordinates
    myNode3D *parent = nullptr;                  // parent node
    myNode3D *neighborhoodCenterNode = nullptr;  // center of the neighborhood (obtained by rollback)
    std::unordered_set<myNode3D *> neighborhood; // path neighborhood set
    bool isCutPoint = false;                     // cut point flag
    int genNo{0};                                // search depth maintained as generation number

    // Updates and checks
    myNode3D *oneStepRollback();
    void findNeighborhoodCenter();
    void getNeighborhood(double nbRadius, int minSearchDepth);
    void cutPointCheck(myNode3D &n);
    void generateCutPointRegion();

    double getDistance(const myNode3D &n) const;

    bool isCoordsEqual(const myNode3D &n) const;   // coordinate comparison
    bool hasNeighborhoodIntersection(myNode3D &n); // neighborhood comparison

    // This must be defined for the node
    bool operator==(myNode3D &n); // comparison

    // constructors
    myNode3D() {}
    myNode3D(COORD_TYPE xx, COORD_TYPE yy, COORD_TYPE zz);

    // Inherited functions being overwritten
    int getHashBin(void) const;
    void print(std::string head = "", std::string tail = "") const;

    // ---------------------------------------------------------------
    // Operator overloading for convex combination -- for Path Recnstruction in SStar algorithm
    myNode3D operator+(const myNode3D &b) const;
    myNode3D operator*(const double &c) const;
};

#endif