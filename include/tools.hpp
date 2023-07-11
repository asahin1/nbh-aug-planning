// Header file for some tools (helper functions and global variables)
// Guards
#ifndef _TOOLS_H
#define _TOOLS_H

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>

// math macros / constants
#define PI 3.14159265359
#define EPS 1E-3

// ---------------------------------------------------
#define COORD_TYPE double

extern double NONUNIFORM_COST_MULTIPLIER;
extern double PLOT_SCALE;

extern double R_HEURISTIC_WEIGHT;
extern int R_ROLLBACK_RADIUS;
extern double R_NEIGHBORHOOD_RADIUS;
extern int R_NEIGHBORHOOD_SEARCH_DEPTH;
extern int R_LINEAGE_DATA_GENERATION_THRESHOLD;
extern bool R_CUT_POINT_CHECK;
extern double R_CP_LOWER_THRESHOLD;
extern double R_CP_UPPER_THRESHOLD;
extern double R_OVERLAP_FOR_CP;
extern double R_CP_PATH_PORTION;
extern double R_CP_GSCORE_DIFF;
extern double R_CPR_RADIUS;

// Merging via looping of unordered sets or unordered maps
template <typename T>
void mergeUnordered(T &l1, T &l2)
{
    T dummyUnordered;
    for (auto &i : l1)
    {
        dummyUnordered.insert(i);
    }
    for (auto &i : l2)
    {
        dummyUnordered.insert(i);
    }
    l1 = dummyUnordered;
    l2 = dummyUnordered;
}
#endif