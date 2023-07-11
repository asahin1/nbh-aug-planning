// Header file for the myNode2D class

// Guards
#ifndef _MYNODE2D_H
#define _MYNODE2D_H

// Standard libraries
#include <unordered_set>

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
class myNode2D : public _DOSL_ALGORITHM::Node<myNode2D, double>
{
public:
    // Attributes
    COORD_TYPE x, y;                                    // coordinates
    myNode2D *parent = nullptr;                         // pointer to parent (at generation time)
    double transition_cost = 0;                         // transition cost from parent to this node
    myNode2D *furthest_grandparent = nullptr;           // pointer to furthest grandparent (updated during predecessor rollback)
    std::unordered_set<myNode2D *> neighborhood;        // set of predecessors (generated during comparison)
    bool isCutPoint = false;                            // merge point flag
    std::unordered_map<myNode2D *, double> parent_list; // set of (parent,transition cost) pairs
    int genNo{0};

    // Plot methods
    void plotNode(CvScalar col = cvScalar(0, 0, 0)); // plot a single node with desired color
    void plotCutPoint();                             // plot as merge point (with orange, show image immediately)

    // Updates and checks
    myNode2D *oneStepRollback();
    void findNeighborhoodCenter();
    void getNeighborhood(double nbRadius, int minGenerations);
    void cutPointCheck(myNode2D &n);
    void generateCutPointRegion();

    bool isCoordsEqual(const myNode2D &n) const;   // coordinate comparison
    bool hasNeighborhoodIntersection(myNode2D &n); // predecessor comparison
    void mergeNodeElements(myNode2D &n);           // merging sets, maps, lists

    double getDistance(const myNode2D &n) const;

    // *** This must be defined for the node
    // Comparison
    bool operator==(myNode2D &n);

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