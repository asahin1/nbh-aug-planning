// Header file for some computation tools (only some global variables for now)

// Guards
#ifndef _TOOLS_H
#define _TOOLS_H

// type of configuration space coordinates
#define COORD_TYPE double

// Algorithm parameters
extern double R_HEURISTIC_WEIGHT;       // omega
extern int R_ROLLBACK_RADIUS;           // r_b
extern double R_NEIGHBORHOOD_RADIUS;    // r_n
extern int R_NEIGHBORHOOD_SEARCH_DEPTH; // Refer to Section 3E
extern bool R_CUT_POINT_CHECK;          // Refer to Section 4B
extern double R_CP_LOWER_THRESHOLD;     // epsilon_lower
extern double R_CP_UPPER_THRESHOLD;     // epsilon_upper
extern double R_OVERLAP_FOR_CP;         // epsilon_i
extern double R_CP_PATH_PORTION;        // r_l
extern double R_CP_GSCORE_DIFF;         // epsilon_g
extern double R_CPR_RADIUS;             // r_MP

#endif