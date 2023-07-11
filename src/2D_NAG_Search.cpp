// Main .cpp file for performing the search and plotting results
// Also includes some function implementations
// =============================================================

// Some libraries (standard)
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
#include <unordered_set>
#include <chrono>
// Other libraries:
// Open CV:
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
// dosl utils
#include <dosl/aux-utils/string_utils.hpp> // compute_program_path
// User made
#include "include/searchProblem2D.hpp" // searchProblem class based on DOSL

searchProblem2D *search2D_ptr{nullptr}; // A searchProblem pointer(NULL for now), required for function implementations

void myNode2D::plotNode(CvScalar col)
{
    search2D_ptr->cvPlotPoint(search2D_ptr->image_to_display, search2D_ptr->cv_plot_coord(x, y), col, PLOT_SCALE);
}

void myNode2D::plotCutPoint()
{
    plotNode(cvScalar(0, 165, 255));
    search2D_ptr->cvPlotPoint(search2D_ptr->cleanMap,
                              search2D_ptr->cv_plot_coord(x, y), cvScalar(0, 165, 255), PLOT_SCALE);
    cv::imshow("Display window", search2D_ptr->image_to_display);
    cv::waitKey(1);
}
// ==============================================================================

void myNode2D::getNeighborhood(double nbRadius, int minGenerations)
{
    findNeighborhoodCenter();
    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    neighborhoodSearchProblem<myNode2D> neighborhood_search(furthest_grandparent, nbRadius, R_HEURISTIC_WEIGHT, minGenerations, &search2D_ptr->startNode, search2D_ptr->image_to_display, search2D_ptr->parsedMap);
    neighborhood_search.search();

    neighborhood = neighborhood_search.collectNeighborhood();
    // neighborhood = neighborhood_search.collectNeighborhoodWithInnerRadius(R_NEIGHBORHOOD_INNER_RADIUS);
}

void myNode2D::cutPointCheck(myNode2D &n)
{
    findNeighborhoodCenter();
    n.findNeighborhoodCenter();
    if (abs(furthest_grandparent->g_score - n.furthest_grandparent->g_score) > R_CP_GSCORE_DIFF)
    {
        return;
    }

    // Run path reconstruction for both
    auto short_path_to_this = search2D_ptr->reconstruct_path_with_length(*furthest_grandparent, furthest_grandparent->g_score * R_CP_PATH_PORTION);
    auto short_path_to_n = search2D_ptr->reconstruct_path_with_length(*n.furthest_grandparent, furthest_grandparent->g_score * R_CP_PATH_PORTION);

    // Get root vertices
    double maxWeight{-1};
    myNode2D *this_root{nullptr};
    for (auto it = short_path_to_this.back().begin(); it != short_path_to_this.back().end(); ++it)
    {
        if (it->second > maxWeight)
            this_root = it->first;
    }
    maxWeight = -1;
    myNode2D *n_root{nullptr};
    for (auto it = short_path_to_n.back().begin(); it != short_path_to_n.back().end(); ++it)
    {
        if (it->second > maxWeight)
            n_root = it->first;
    }

    if (this_root == n_root)
        return;
    cutPointSearchProblem<myNode2D> cpSearch(this_root, R_CP_UPPER_THRESHOLD, R_NEIGHBORHOOD_SEARCH_DEPTH, n_root, &search2D_ptr->startNode, search2D_ptr->image_to_display, search2D_ptr->parsedMap);
    cpSearch.search();

    if (cpSearch.reachedGoal && cpSearch.pathLength > R_CP_LOWER_THRESHOLD)
    {
        isCutPoint = true;
        n.isCutPoint = true;
        plotCutPoint();
        n.plotCutPoint();
        generateCutPointRegion();
        for (auto &s : n.parent->successors)
        {
            s.first->isCutPoint = true;
        }
    }
}

void myNode2D::generateCutPointRegion()
{
    findNeighborhoodCenter();
    neighborhoodSearchProblem<myNode2D> cp_region_search(furthest_grandparent, R_CPR_RADIUS, 0, 0, &search2D_ptr->startNode, search2D_ptr->image_to_display, search2D_ptr->parsedMap);
    cp_region_search.search();

    auto cp_region = cp_region_search.collectNeighborhood();
    for (auto &n : cp_region)
    {
        n->isCutPoint = true;
        n->plotCutPoint();
        // cv::waitKey(0);
    }
}

// Main function for constructing and running the search problem and generating the results
int main(int argc, char *argv[])
{
    compute_program_path();

    std::string expt_f_name = program_path + "../expt/2d_environments.json", expt_name = "map_indoor";
    std::string param_f_name = program_path + "../expt/algorithm_parameters.json";
    std::string param_setName = "2d_original";

    if (argc == 2)
    {
        expt_name = argv[1];
    }
    else if (argc > 2)
    {
        expt_name = argv[1];
        param_setName = argv[2];
    }

    searchProblem2D search2D_inst(expt_f_name, expt_name, param_f_name, param_setName);
    search2D_ptr = &search2D_inst;
    search2D_inst.start_time = std::chrono::steady_clock::now();
    search2D_inst.search();
    search2D_inst.end_time = std::chrono::steady_clock::now();
    std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::seconds>(search2D_inst.end_time - search2D_inst.start_time).count() << "s" << std::endl;

    int valid_paths = 0;
    std::vector<std::vector<myNode2D>> valid_path_vec;
    std::vector<double> valid_path_costs;
    std::vector<std::vector<myNode2D>> invalid_path_vec;
    std::vector<double> invalid_path_costs;

    for (int i = 0; i < search2D_inst.foundGoals.size(); ++i)
    {
        // get path
        auto path = search2D_inst.reconstruct_weighted_pointer_path(search2D_inst.foundGoals[i]);
        double cost = 0.0;
        bool containsMergePoint = false;
        myNode2D thisPt = search2D_inst.startNode, lastPt;
        std::vector<myNode2D> path_vec;
        for (int a = path.size() - 1; a >= 0; --a)
        {
            lastPt = thisPt;
            thisPt = myNode2D(0.0, 0.0);
            for (auto it = path[a].begin(); it != path[a].end(); ++it)
            {
                if (it->first->isCutPoint)
                {
                    containsMergePoint = true;
                }
                thisPt.x += it->second * it->first->x;
                thisPt.y += it->second * it->first->y;
            }
            path_vec.push_back(thisPt);
        }
        if (containsMergePoint)
        {
            invalid_path_vec.push_back(path_vec);
            invalid_path_costs.push_back(search2D_inst.foundGoals[i].g_score);
        }
        else
        {
            valid_paths++;
            valid_path_vec.push_back(path_vec);
            valid_path_costs.push_back(search2D_inst.foundGoals[i].g_score);
        }
    }

    std::cout << ((valid_paths == search2D_ptr->foundGoals.size()) ? "" : "Only ") << valid_paths << " valid path"
              << ((valid_paths != 1) ? "s " : " ")
              << "found out of "
              << search2D_ptr->nPathsToFind << " desired path"
              << ((search2D_ptr->nPathsToFind != 1) ? "s." : ".") << std::endl;

    for (int i{0}; i < valid_path_vec.size(); i++)
    {
        myNode2D p2 = search2D_inst.startNode, p1;
        for (int a = 1; a < valid_path_vec[i].size(); a++)
        {
            p1 = p2;
            p2 = valid_path_vec[i][a];
            cv::line(search2D_inst.image_to_display,
                     search2D_inst.cv_plot_coord(p2.x, p2.y), search2D_inst.cv_plot_coord(p1.x, p1.y),
                     cvScalar(102.0, 0.0, 204.0),
                     search2D_inst.LINE_THICKNESS * PLOT_SCALE * 0.5);
            cv::line(search2D_inst.cleanMap,
                     search2D_inst.cv_plot_coord(p2.x, p2.y), search2D_inst.cv_plot_coord(p1.x, p1.y),
                     cvScalar(0.0, 102.0, 0.0),
                     search2D_inst.LINE_THICKNESS * PLOT_SCALE * 0.5);
        }
        std::cout << "Goal g-score (accounts for nonuniform path cost): " << valid_path_costs[i] << std::endl;

        cv::imshow("Display window", search2D_inst.image_to_display);
        char key = (char)cv::waitKey(); // explicit cast
        while (key != 27)
        {
            key = (char)cv::waitKey();
        }
    }

    for (int i{0}; i < invalid_path_vec.size(); i++)
    {
        myNode2D p2 = search2D_inst.startNode, p1;
        for (int a = 1; a < invalid_path_vec[i].size(); a++)
        {
            p1 = p2;
            p2 = invalid_path_vec[i][a];
            cv::line(search2D_inst.image_to_display,
                     search2D_inst.cv_plot_coord(p2.x, p2.y), search2D_inst.cv_plot_coord(p1.x, p1.y),
                     cvScalar(255, 153, 51),
                     search2D_inst.LINE_THICKNESS * PLOT_SCALE * 0.5);
            cv::line(search2D_inst.cleanMap,
                     search2D_inst.cv_plot_coord(p2.x, p2.y), search2D_inst.cv_plot_coord(p1.x, p1.y),
                     cvScalar(153, 51, 255),
                     search2D_inst.LINE_THICKNESS * PLOT_SCALE * 0.5);
        }
        std::cout << "Goal g-score (accounts for nonuniform path cost): " << invalid_path_costs[i] << std::endl;

        cv::imshow("Display window", search2D_inst.image_to_display);
        char key = (char)cv::waitKey(); // explicit cast
        while (key != 27)
        {
            key = (char)cv::waitKey();
        }
    }
    search2D_inst.cvPlotStartGoal(search2D_inst.cleanMap);
    cv::imshow("Display window", search2D_inst.image_to_display);
    cv::imshow("Original window", search2D_inst.cleanMap);
    char key = (char)cv::waitKey(); // explicit cast
    while (key != 27)
    {
        key = (char)cv::waitKey();
    }
}
