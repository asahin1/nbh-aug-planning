// Header file for the myNode class, for now class implementations are included
// Guards
#ifndef _3D_MYNODE_H
#define _3D_MYNODE_H

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
#include "create_neighborhood.cpp" // coded by SB
#include "mergePointSearch.cpp"
#include "tools.hpp" // some global variables and helper functions

// Declaration of myNode class
class myNode3D : public _DOSL_ALGORITHM::Node<myNode3D, double>
{
public:
    COORD_TYPE x, y, z;                                 // coordinates
    myNode3D *parent = nullptr;                         // pointer to parent (at generation time)
    double transition_cost = 0;                         // transition cost from parent to this node
    myNode3D *furthest_grandparent = nullptr;           // pointer to furthest grandparent (updated during predecessor rollback)
    std::unordered_set<myNode3D *> neighborhood;        // set of predecessors (generated during comparison)
    bool isMergePoint = false;                          // merge point flag
    std::unordered_map<myNode3D *, double> parent_list; // set of (parent,transition cost) pairs

    int genNo{0};

    // Plot methods
    void plotNode(std::vector<double> col = {0, 0, 0});                 // plot a single node with desired color
    void plotNodeSet(std::unordered_set<myNode3D *> nodeSet,            //
                     std::vector<double> col = {0.0, 0.0, 1.0});        // plot a set of nodes with desired color
    void plotMergePoint();                                              // plot as merge point (with orange, show image immediately)
    void plotNeighborhood(std::vector<double> col = {1.0, 0.6, 0.6},    //
                          bool restore = false);                        // Plots all predecessors (with given color)
    void plotCommonNodes(std::unordered_set<myNode3D *> commonNodes,    //
                         bool pause = false, bool restore = false);     // plot nodes at neighborhood intersection
    void plotNeighborhoods(myNode3D *otherNode,                         //
                           std::unordered_set<myNode3D *> commonNodes); // plot neighborhoods for two nodes
    void plotMPNBs(myNode3D *otherNode);

    // Print methods
    void printNeighborhood() const;
    void printSuccessors() const;
    void printParents() const;
    void printCameFromSimplex() const;

    // Updates and checks
    myNode3D *oneStepRollback();
    void findFurthestGrandParent();
    void getNeighborhood(double nbRadius, int minGenerations);
    void getMergePointSets(double lowerRadius, double upperRadius, int minGenerations);
    void mergePointCheck(myNode3D &n);
    void alternativeMergePointCheck(myNode3D &n);
    void generateMergePointRegion();

    bool isCoordsEqual(const myNode3D &n) const;      // coordinate comparison
    bool hasNeighborhoodIntersection(myNode3D &n);    // predecessor comparison
    void mergeNodeElements(myNode3D &n);              // merging sets, maps, lists
    void reconstructPathToNode(bool restore = false); // reconstruct and plot path to this node

    double getDist(const myNode3D &n) const;
    int getNumberOfCopies() const;

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

    myNode3D operator+(const myNode &b) const;

    myNode3D operator*(const double &c) const;
};

#endif