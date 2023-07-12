// Header file for some computation tools (only some global variables for now)

// Guards
#ifndef _TOOLS_H
#define _TOOLS_H

// type of configuration space coordinates
#define COORD_TYPE double

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

#endif