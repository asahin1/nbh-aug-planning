// Main .cpp file for performing the search and plotting results
// Also includes some function implementations
// =============================================================
// dosl utils
#include <dosl/aux-utils/string_utils.hpp> // compute_program_path
// User made
#include "include/searchProblem2D.hpp" // searchProblem class based on DOSL

searchProblem2D *search2D_ptr{nullptr}; // A searchProblem pointer(NULL for now), required for function implementations

void myNode2D::getNeighborhood(double nbRadius, int minSearchDepth)
{
    findNeighborhoodCenter();
    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    neighborhoodSearchProblem<myNode2D> neighborhood_search(neighborhoodCenterNode, nbRadius, R_HEURISTIC_WEIGHT, minSearchDepth, &search2D_ptr->startNode, search2D_ptr->parsedMap);
    neighborhood_search.search();

    neighborhood = neighborhood_search.collectNeighborhood();
    // neighborhood = neighborhood_search.collectNeighborhoodWithInnerRadius(R_NEIGHBORHOOD_INNER_RADIUS);
}

void myNode2D::cutPointCheck(myNode2D &n)
{
    findNeighborhoodCenter();
    n.findNeighborhoodCenter();
    if (abs(neighborhoodCenterNode->g_score - n.neighborhoodCenterNode->g_score) > R_CP_GSCORE_DIFF)
    {
        return;
    }

    // Run path reconstruction for both
    auto short_path_to_this = search2D_ptr->reconstruct_path_with_length(*neighborhoodCenterNode, neighborhoodCenterNode->g_score * R_CP_PATH_PORTION);
    auto short_path_to_n = search2D_ptr->reconstruct_path_with_length(*n.neighborhoodCenterNode, neighborhoodCenterNode->g_score * R_CP_PATH_PORTION);

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
    cutPointSearchProblem<myNode2D> cpSearch(this_root, R_CP_UPPER_THRESHOLD, n_root, &search2D_ptr->startNode, search2D_ptr->parsedMap);
    cpSearch.search();

    if (cpSearch.reachedGoal && cpSearch.pathLength > R_CP_LOWER_THRESHOLD)
    {
        isCutPoint = true;
        n.isCutPoint = true;
        search2D_ptr->plotCutPoint(*this);
        // plotCutPoint();
        search2D_ptr->plotCutPoint(n);
        // n.plotCutPoint();
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
    neighborhoodSearchProblem<myNode2D> cp_region_search(neighborhoodCenterNode, R_CPR_RADIUS, 0, 0, &search2D_ptr->startNode, search2D_ptr->parsedMap);
    cp_region_search.search();

    auto cp_region = cp_region_search.collectNeighborhood();
    for (auto &n : cp_region)
    {
        n->isCutPoint = true;
        search2D_ptr->plotCutPoint(*n);
        // n->plotCutPoint();
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
    search2D_inst.popUpText("Search complete. Press ESC to view paths.");
    std::cout << "=============================================================" << std::endl;
    std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::seconds>(search2D_inst.end_time - search2D_inst.start_time).count() << "s" << std::endl;

    for (int i = 0; i < search2D_inst.foundGoals.size(); ++i)
    {
        // get path
        auto path = search2D_inst.reconstruct_weighted_pointer_path(search2D_inst.foundGoals[i]);
        search2D_inst.plotPath(search2D_inst.image_to_display, path, cvScalar(102.0, 0.0, 204.0));
        search2D_inst.plotPath(search2D_inst.cleanMap, path, cvScalar(0.0, 102.0, 0.0));
        cv::imshow("Display window", search2D_inst.image_to_display);
        char key = (char)cv::waitKey(); // explicit cast
        while (key != 27)
        {
            key = (char)cv::waitKey();
        }
        std::cout << "Cost of Path #" << i + 1 << ": " << search2D_inst.foundGoals[i].g_score << std::endl;
    }

    std::cout << ((search2D_inst.nPathsToFind == search2D_inst.nPathsFound) ? "" : "Only ") << search2D_inst.nPathsFound << " path"
              << ((search2D_inst.nPathsFound != 1) ? "s " : " ")
              << "found out of "
              << search2D_inst.nPathsToFind << " desired path"
              << ((search2D_inst.nPathsToFind != 1) ? "s." : ".") << std::endl;

    search2D_inst.cvPlotStartGoal(search2D_inst.cleanMap);
    cv::imshow("Display window", search2D_inst.image_to_display);
    cv::imshow("Original window", search2D_inst.cleanMap);
    char key = (char)cv::waitKey(); // explicit cast
    while (key != 27)
    {
        key = (char)cv::waitKey();
    }
    std::cout << "End of program." << std::endl;
    std::cout << "=============================================================" << std::endl;
    return 0;
}
