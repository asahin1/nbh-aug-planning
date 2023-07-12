// Main .cpp file for performing the search and plotting results
// Also includes some function implementations
// =============================================================
// #include <chrono>
#include <dosl/aux-utils/string_utils.hpp> // compute_program_path
#include "include/searchProblem3D.hpp"     // searchProblem class based on DOSL
// =========================================================================================================================================================
searchProblem3D *search3D_ptr{nullptr};
#define bool bool
// ==============================================================================

void myNode3D::getNeighborhood(double nbRadius, int minSearchDepth)
{
    findNeighborhoodCenter();
    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    neighborhoodSearchProblem<myNode3D> neighborhood_search(neighborhoodCenterNode, nbRadius, R_HEURISTIC_WEIGHT, minSearchDepth, &search3D_ptr->startNode, search3D_ptr->occupancyMap);
    neighborhood_search.search();

    neighborhood = neighborhood_search.collectNeighborhood();
    // neighborhood = neighborhood_search.collectNeighborhoodWithInnerRadius(R_NEIGHBORHOOD_INNER_RADIUS);
}

void myNode3D::cutPointCheck(myNode3D &n)
{
    findNeighborhoodCenter();
    n.findNeighborhoodCenter();
    if (abs(neighborhoodCenterNode->g_score - n.neighborhoodCenterNode->g_score) > R_CP_GSCORE_DIFF)
        return;

    // std::cout << "this_p_l: " << furthest_grandparent->g_score << "n_p_l: " << n.furthest_grandparent->g_score << std::endl;

    // Run path reconstruction for both
    auto short_path_to_this = search3D_ptr->reconstruct_path_with_length(*neighborhoodCenterNode, neighborhoodCenterNode->g_score * R_CP_PATH_PORTION);
    auto short_path_to_n = search3D_ptr->reconstruct_path_with_length(*n.neighborhoodCenterNode, neighborhoodCenterNode->g_score * R_CP_PATH_PORTION);

    // Get root vertices
    double maxWeight{-1};
    myNode3D *this_root{nullptr};
    for (auto it = short_path_to_this.back().begin(); it != short_path_to_this.back().end(); ++it)
    {
        if (it->second > maxWeight)
            this_root = it->first;
    }
    maxWeight = -1;
    myNode3D *n_root{nullptr};
    for (auto it = short_path_to_n.back().begin(); it != short_path_to_n.back().end(); ++it)
    {
        if (it->second > maxWeight)
            n_root = it->first;
    }

    if (this_root == n_root)
        return;

    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    cutPointSearchProblem<myNode3D> cpSearch(this_root, R_CP_UPPER_THRESHOLD, n_root, &search3D_ptr->startNode, search3D_ptr->occupancyMap);
    cpSearch.search();

    if (cpSearch.reachedGoal && cpSearch.pathLength > R_CP_LOWER_THRESHOLD)
    // if (!mpSearch.reachedGoal)
    {
        std::cout << "Path length: " << cpSearch.pathLength << std::endl;
        isCutPoint = true;
        n.isCutPoint = true;
        search3D_ptr->plotCutPoint(*this);
        // plotCutPoint();
        search3D_ptr->plotCutPoint(n);
        // n.plotCutPoint();
        generateCutPointRegion();
        for (auto &s : n.parent->successors)
        {
            s.first->isCutPoint = true;
        }
    }
}

void myNode3D::generateCutPointRegion()
{
    findNeighborhoodCenter();
    neighborhoodSearchProblem<myNode3D> cp_region_search(neighborhoodCenterNode, R_CPR_RADIUS, 0, 0, &search3D_ptr->startNode, search3D_ptr->occupancyMap);
    cp_region_search.search();

    auto cp_region = cp_region_search.collectNeighborhood();
    for (auto &n : cp_region)
    {
        n->isCutPoint = true;
        search3D_ptr->plotCutPoint(*n);
        // n->plotCutPoint();
    }
}

// Main function for constructing and running the search problem and generating the results
int main(int argc, char *argv[])
{
    // RUNTIME_VERBOSE_SWITCH = 0;
    compute_program_path();

    std::string expt_f_name = program_path + "../expt/3d_environments.json", expt_name = "env_genus_1";
    std::string param_f_name = program_path + "../expt/algorithm_parameters.json";
    std::string param_setName = "3d_original";
    if (argc == 2)
    {
        expt_name = argv[1];
    }
    else if (argc > 2)
    {
        expt_name = argv[1];
        param_setName = argv[2];
    }

    searchProblem3D search3D_inst(expt_f_name, expt_name, param_f_name, param_setName);
    search3D_ptr = &search3D_inst;
    search3D_inst.start_time = std::chrono::steady_clock::now();
    search3D_inst.search();
    search3D_inst.end_time = std::chrono::steady_clock::now();
    std::cout << "Search complete. Press ESC to view paths." << std::endl;
    std::cout << "=============================================================" << std::endl;
    std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::seconds>(search3D_inst.end_time - search3D_inst.start_time).count() << "s" << std::endl;

    search3D_inst.figure_to_display.get_key();
    for (int i{search3D_inst.MIN_X}; i < search3D_inst.MAX_X + 1; i++)
    {
        for (int j{search3D_inst.MIN_Y}; j < search3D_inst.MAX_Y + 1; j++)
        {
            for (int k{search3D_inst.MIN_Z}; k < search3D_inst.MAX_Z + 1; k++)
            {
                auto color_at_box = search3D_inst.displayBoxes.at(i).at(j).at(k)->color();
                search3D_inst.displayBoxes.at(i).at(j).at(k)->visible() = (color_at_box == sglMake3vec(1, 0.6, 0));
                // if (test_search_problem.displayBoxes.at(i).at(j).at(k)->visible())
                //     color_at_box = sglMake3vec(1, 0.64706, 0);
            }
        }
    }
    search3D_inst.figure_to_display.flush();
    for (int i = 0; i < search3D_inst.foundGoals.size(); ++i)
    {
        // get path
        auto path = search3D_inst.reconstruct_weighted_pointer_path(search3D_inst.foundGoals[i]);
        search3D_inst.plotPath(path, {0.4, 0, 0.9});
        search3D_inst.figure_to_display.flush();
        search3D_inst.figure_to_display.get_key();
        std::cout << "Cost of Path #" << i + 1 << ": " << search3D_inst.foundGoals[i].g_score << std::endl;
    }

    std::cout << ((search3D_inst.nPathsToFind == search3D_inst.nPathsFound) ? "" : "Only ") << search3D_inst.nPathsFound << " path"
              << ((search3D_inst.nPathsFound != 1) ? "s " : " ")
              << "found out of "
              << search3D_inst.nPathsToFind << " desired path"
              << ((search3D_inst.nPathsToFind != 1) ? "s." : ".") << std::endl;

    _dosl_cout << "Press any key to quit..." << _dosl_endl;
    search3D_inst.figure_to_display.get_key();
    std::cout << "End of program." << std::endl;
    std::cout << "=============================================================" << std::endl;
    return 0;
}