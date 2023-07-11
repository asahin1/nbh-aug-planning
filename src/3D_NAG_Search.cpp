// Main .cpp file for performing the search and plotting results
// Also includes some function implementations
// =============================================================
#include <chrono>
#include <dosl/aux-utils/string_utils.hpp> // compute_program_path
#include "include/searchProblem3D.hpp"     // searchProblem class based on DOSL
// =========================================================================================================================================================
searchProblem *my_search_problem = NULL;
#define bool bool

void myNode::plotNode(std::vector<double> col)
{
    my_search_problem->displayBoxes.at(x).at(y).at(z)->color() = col;
    my_search_problem->displayBoxes.at(x).at(y).at(z)->visible() = true;
}

void myNode::plotMergePoint()
{
    plotNode({1, 0.6, 0});
    my_search_problem->my_fig.flush();
    sleep(0.5);
}

void myNode::plotNodeSet(std::unordered_set<myNode *> nodeSet, std::vector<double> col)
{
    for (auto it = nodeSet.begin(); it != nodeSet.end(); it++)
    {
        (*it)->plotNode(col);
    }
}

// Some myNode::function implementations
void myNode::plotNeighborhood(std::vector<double> col, bool restore)
{ // plotting the predecessor list
    cv::Mat temp_img = my_search_problem->image_to_display.clone();
    plotNodeSet(neighborhood, col);
    // node itself with pink color
    plotNode({1, 0, 1});
    // neighborhood center with yellow
    if (furthest_grandparent)
        furthest_grandparent->plotNode({0.0, 1.0, 1.0});

    my_search_problem->my_fig.flush();
    sleep(0.5);
    if (restore)
        my_search_problem->image_to_display = temp_img;
}

void myNode::plotCommonNodes(std::unordered_set<myNode *> commonNodes, bool pause, bool restore)
{
    cv::Mat temp_img = my_search_problem->image_to_display.clone();
    plotNodeSet(commonNodes, {1.0, 1.0, 0.4});
    cv::imshow("Display window", my_search_problem->image_to_display);
    cv::waitKey(1);
    if (restore)
        my_search_problem->image_to_display = temp_img;
}

void myNode::plotNeighborhoods(myNode *otherNode, std::unordered_set<myNode *> commonNodes)
{
    cv::Mat temp_img = my_search_problem->image_to_display.clone();
    plotNeighborhood({0, 0, 1}, false);
    otherNode->plotNeighborhood({1, 0, 0}, false);
    plotCommonNodes(commonNodes, true, false);
    cv::waitKey(); // explicit cast
    furthest_grandparent->reconstructPathToNode();
    otherNode->furthest_grandparent->reconstructPathToNode();
    my_search_problem->image_to_display = temp_img;
}

void myNode::plotMPNBs(myNode *otherNode)
{
    cv::Mat temp_img = my_search_problem->image_to_display.clone();
    plotNodeSet(mp_upper_nb, {255, 102, 102});
    cv::imshow("Display window", my_search_problem->image_to_display);
    cv::waitKey(); // explicit cast
    plotNodeSet(otherNode->mp_upper_nb, {102, 102, 255});
    cv::imshow("Display window", my_search_problem->image_to_display);
    cv::waitKey(); // explicit cast
    plotNodeSet(mp_lower_nb, {153, 0, 0});
    cv::imshow("Display window", my_search_problem->image_to_display);
    cv::waitKey(); // explicit cast
    plotNodeSet(otherNode->mp_lower_nb, {0, 0, 153});
    cv::imshow("Display window", my_search_problem->image_to_display);
    cv::waitKey(); // explicit cast
    furthest_grandparent->reconstructPathToNode();
    otherNode->furthest_grandparent->reconstructPathToNode();
    my_search_problem->image_to_display = temp_img;
}

void myNode::reconstructPathToNode(bool restore)
{
    // // temporary storage for current display
    // cv::Mat temp_store_curr_img = my_search_problem->image_to_display.clone(); // store the current state of image_to_display
    // // get path
    // auto path = my_search_problem->reconstruct_weighted_pointer_path(*this);
    // myNode thisPt = my_search_problem->startNode, lastPt;
    // std::vector<myNode> allPts;
    // for (int a = path.size() - 1; a >= 0; --a)
    // {
    //     lastPt = thisPt;
    //     thisPt = myNode(0.0, 0.0, 0.0);
    //     for (auto it = path[a].begin(); it != path[a].end(); ++it)
    //     {
    //         thisPt.x += it->second * it->first->x;
    //         thisPt.y += it->second * it->first->y;
    //         thisPt.z += it->second * it->first->z;
    //     }
    //     allPts.push_back(thisPt);
    //     cv::line(my_search_problem->image_to_display,
    //              my_search_problem->cv_plot_coord(thisPt.x, thisPt.y), my_search_problem->cv_plot_coord(lastPt.x, lastPt.y),
    //              {200.0, 100.0, 100.0},
    //              my_search_problem->LINE_THICKNESS * PLOT_SCALE * 0.2);
    // }

    // cv::imshow("Display window", my_search_problem->image_to_display);
    // char key = (char)cv::waitKey(); // explicit cast
    // while (key != 27)
    // {
    //     key = (char)cv::waitKey();
    // }
    // if (restore)
    //     my_search_problem->image_to_display = temp_store_curr_img.clone();
}

void plotPath(std::vector<myNode *> path, bool restore)
{
    // // temporary storage for current display
    // cv::Mat temp_store_curr_img = my_search_problem->image_to_display.clone(); // store the current state of image_to_display
    // myNode thisPt = *path.back(), lastPt;
    // double cost = 0.0;
    // for (int a = path.size() - 1; a >= 0; --a)
    // {
    //     lastPt = thisPt;
    //     thisPt = *path[a];
    //     cv::line(my_search_problem->image_to_display,
    //              my_search_problem->cv_plot_coord(thisPt.x, thisPt.y), my_search_problem->cv_plot_coord(lastPt.x, lastPt.y),
    //              {100.0, 100.0, 200.0},
    //              my_search_problem->LINE_THICKNESS * PLOT_SCALE * 0.2);
    //     cost += sqrt((thisPt.x - lastPt.x) * (thisPt.x - lastPt.x) + (thisPt.y - lastPt.y) * (thisPt.y - lastPt.y));
    // }
    // printf("\tPath cost: %f.(without cost multiplier)\n", cost);
    // cv::imshow("Display window", my_search_problem->image_to_display);
    // char key = (char)cv::waitKey(); // explicit cast
    // while (key != 27)
    // {
    //     key = (char)cv::waitKey();
    // }
    // if (restore)
    //     my_search_problem->image_to_display = temp_store_curr_img.clone();
}

void plotPath(std::vector<std::unordered_map<myNode *, double>> path, bool restore)
{
    // // temporary storage for current display
    // cv::Mat temp_store_curr_img = my_search_problem->image_to_display.clone(); // store the current state of image_to_display
    // myNode lastPt;
    // myNode thisPt(0.0, 0.0);
    // for (auto it = path.back().begin(); it != path.back().end(); ++it)
    // {
    //     thisPt.x += it->second * it->first->x;
    //     thisPt.y += it->second * it->first->y;
    // }
    // double cost = 0.0;
    // for (int a = path.size() - 1; a >= 0; --a)
    // {
    //     lastPt = thisPt;
    //     thisPt = myNode(0.0, 0.0);
    //     for (auto it = path[a].begin(); it != path[a].end(); ++it)
    //     {
    //         thisPt.x += it->second * it->first->x;
    //         thisPt.y += it->second * it->first->y;
    //     }
    //     cv::line(my_search_problem->image_to_display,
    //              my_search_problem->cv_plot_coord(thisPt.x, thisPt.y), my_search_problem->cv_plot_coord(lastPt.x, lastPt.y),
    //              {100.0, 100.0, 200.0},
    //              my_search_problem->LINE_THICKNESS * PLOT_SCALE * 0.2);
    //     cost += sqrt((thisPt.x - lastPt.x) * (thisPt.x - lastPt.x) + (thisPt.y - lastPt.y) * (thisPt.y - lastPt.y));
    // }
    // printf("\tPath cost: %f.(without cost multiplier)\n", cost);
    // cv::imshow("Display window", my_search_problem->image_to_display);
    // char key = (char)cv::waitKey(); // explicit cast
    // while (key != 27)
    // {
    //     key = (char)cv::waitKey();
    // }
    // if (restore)
    //     my_search_problem->image_to_display = temp_store_curr_img.clone();
}

// ==============================================================================

void myNode::getNeighborhood(double nbRadius, int minGenerations)
{
    findFurthestGrandParent();
    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    neighborhoodSearchProblem<myNode> neighborhood_search(furthest_grandparent, nbRadius, R_HEURISTIC_WEIGHT, minGenerations, &my_search_problem->startNode, my_search_problem->image_to_display, my_search_problem->occupancyMap);
    neighborhood_search.search();

    neighborhood = neighborhood_search.collectNeighborhood();
    // neighborhood = neighborhood_search.collectNeighborhoodWithInnerRadius(R_NEIGHBORHOOD_INNER_RADIUS);
}

void myNode::getMergePointSets(double lowerRadius, double upperRadius, int minGenerations)
{
    findFurthestGrandParent();
    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    neighborhoodSearchProblem<myNode> neighborhood_search(furthest_grandparent, upperRadius, 0, minGenerations, &my_search_problem->startNode, my_search_problem->image_to_display, my_search_problem->occupancyMap);
    neighborhood_search.search();

    mp_upper_nb = neighborhood_search.collectNeighborhood();
    mp_lower_nb = neighborhood_search.collectNeighborhoodWithRadius(lowerRadius);
}

void myNode::alternativeMergePointCheck(myNode &n)
{
    // plotNode({0, 0, 0));
    // cv::imshow("Display window", my_search_problem->image_to_display);
    // cv::waitKey(1);
    // Find the vertices to run the path reconstruction to
    findFurthestGrandParent();
    n.findFurthestGrandParent();
    if (abs(furthest_grandparent->g_score - n.furthest_grandparent->g_score) > R_MP_GSCORE_DIFF)
        return;

    // std::cout << "this_p_l: " << furthest_grandparent->g_score << "n_p_l: " << n.furthest_grandparent->g_score << std::endl;

    // Run path reconstruction for both
    auto short_path_to_this = my_search_problem->reconstruct_path_with_length(*furthest_grandparent, furthest_grandparent->g_score * R_MP_PATH_LENGTH);
    auto short_path_to_n = my_search_problem->reconstruct_path_with_length(*n.furthest_grandparent, furthest_grandparent->g_score * R_MP_PATH_LENGTH);
    if (short_path_to_this.size() == 0 || short_path_to_n.size() == 0)
        return;
    // auto dummy_short_path_to_this = my_search_problem->reconstruct_weighted_pointer_path(*furthest_grandparent);
    // auto dummy_short_path_to_n = my_search_problem->reconstruct_weighted_pointer_path(*n.furthest_grandparent);

    // std::vector<std::unordered_map<myNode *, double>> short_path_to_this;
    // double curr_length = 0;
    // myNode p1(0, 0, 0);
    // for (auto it1 = dummy_short_path_to_this[0].begin(); it1 != dummy_short_path_to_this[0].end(); ++it1)
    // {
    //     p1.x += it1->second * it1->first->x;
    //     p1.y += it1->second * it1->first->y;
    //     p1.z += it1->second * it1->first->z;
    // }
    // for (int i{1}; i < dummy_short_path_to_this.size(); i++)
    // {
    //     if (curr_length > furthest_grandparent->g_score * R_MP_PATH_LENGTH)
    //     {
    //         break;
    //     }
    //     short_path_to_this.push_back(dummy_short_path_to_this[i - 1]);
    //     myNode p2(0, 0, 0);
    //     for (auto it2 = dummy_short_path_to_this[i].begin(); it2 != dummy_short_path_to_this[i].end(); ++it2)
    //     {
    //         p2.x += it2->second * it2->first->x;
    //         p2.y += it2->second * it2->first->y;
    //         p2.z += it2->second * it2->first->z;
    //     }
    //     curr_length += p1.getDist(p2);
    //     p1 = p2;
    // }
    // std::vector<std::unordered_map<myNode *, double>> short_path_to_n;
    // curr_length = 0;
    // p1 = myNode(0, 0, 0);
    // for (auto it1 = dummy_short_path_to_n[0].begin(); it1 != dummy_short_path_to_n[0].end(); ++it1)
    // {
    //     p1.x += it1->second * it1->first->x;
    //     p1.y += it1->second * it1->first->y;
    //     p1.z += it1->second * it1->first->z;
    // }
    // for (int i{1}; i < dummy_short_path_to_n.size(); i++)
    // {
    //     if (curr_length > furthest_grandparent->g_score * R_MP_PATH_LENGTH)
    //     {
    //         break;
    //     }
    //     short_path_to_n.push_back(dummy_short_path_to_n[i - 1]);
    //     myNode p2(0, 0, 0);
    //     for (auto it2 = dummy_short_path_to_n[i].begin(); it2 != dummy_short_path_to_n[i].end(); ++it2)
    //     {
    //         p2.x += it2->second * it2->first->x;
    //         p2.y += it2->second * it2->first->y;
    //         p2.z += it2->second * it2->first->z;
    //     }
    //     curr_length += p1.getDist(p2);
    //     p1 = p2;
    // }

    // Get root vertices
    double maxWeight{-1};
    myNode *this_root{nullptr};
    for (auto it = short_path_to_this.back().begin(); it != short_path_to_this.back().end(); ++it)
    {
        if (it->second > maxWeight)
            this_root = it->first;
    }
    maxWeight = -1;
    myNode *n_root{nullptr};
    for (auto it = short_path_to_n.back().begin(); it != short_path_to_n.back().end(); ++it)
    {
        if (it->second > maxWeight)
            n_root = it->first;
    }

    if (this_root == n_root)
        return;

    // if (y == 50)
    // {
    //     cv::Mat temp_img = my_search_problem->image_to_display.clone();
    //     this_root->plotNode({255, 102, 102));
    //     n_root->plotNode({102, 102, 255));
    //     this_root->reconstructPathToNode();
    //     n_root->reconstructPathToNode();
    //     plotPath(short_path_to_this, false);
    //     plotPath(short_path_to_n, false);
    //     my_search_problem->image_to_display = temp_img;
    // }

    // SB: creating neighborhood centered on the furthest grandparent (then assign it to predecessors)
    mergePointSearchProblem<myNode> mpSearch(this_root, R_MP_UPPER_THRESHOLD, R_MIN_NB_GENERATIONS, n_root, &my_search_problem->startNode, my_search_problem->image_to_display, my_search_problem->occupancyMap);
    mpSearch.search();
    // std::cout << "Reached goal: " << mpSearch.reachedGoal << std::endl;
    // std::cout << "Path length: " << mpSearch.pathLength << std::endl;

    // if (mpSearch.reachedGoal && mpSearch.pathLength <= R_MP_LOWER_THRESHOLD && y == 50 && x > 65)
    // if (mpSearch.reachedGoal && mpSearch.pathLength <= R_MP_LOWER_THRESHOLD && y == 50)
    // {
    //     std::cout << "Path length: " << mpSearch.pathLength << std::endl;
    //     if (R_PLOT_MP_NB)
    //     {
    //         auto path = mpSearch.getPath();
    //         cv::Mat temp_img = my_search_problem->image_to_display.clone();
    //         plotNode({0, 255, 255));
    //         // furthest_grandparent->plotNode({255, 102, 102));
    //         // n.furthest_grandparent->plotNode({102, 102, 255));
    //         // furthest_grandparent->reconstructPathToNode();
    //         // n.furthest_grandparent->reconstructPathToNode();
    //         this_root->plotNode({255, 102, 102));
    //         n_root->plotNode({102, 102, 255));
    //         this_root->reconstructPathToNode();
    //         n_root->reconstructPathToNode();
    //         plotPath(path, false);
    //         my_search_problem->image_to_display = temp_img;
    //     }
    // }
    // if (R_PLOT_MP_NB && y == 50 && x > 50)
    // {
    //     print("This node: ");
    //     std::cout << "Reached goal: " << mpSearch.reachedGoal << std::endl;
    //     std::cout << "Path length: " << mpSearch.pathLength << std::endl;
    //     cv::Mat temp_img = my_search_problem->image_to_display.clone();
    //     plotNode({0, 0, 0));
    //     this_root->plotNode({255, 102, 102));
    //     n_root->plotNode({102, 102, 255));
    //                 this_root->reconstructPathToNode();
    //                 n_root->reconstructPathToNode();
    //                 my_search_problem->image_to_display = temp_img;
    // }

    if (mpSearch.reachedGoal && mpSearch.pathLength > R_MP_LOWER_THRESHOLD)
    // if (!mpSearch.reachedGoal)
    {
        std::cout << "Path length: " << mpSearch.pathLength << std::endl;
        isMergePoint = true;
        n.isMergePoint = true;
        generateMergePointRegion();
        plotMergePoint();
        n.plotMergePoint();
        for (auto &s : n.parent->successors)
        {
            s.first->isMergePoint = true;
        }
        // if (R_PLOT_MP_NB)
        // {
        //     // auto path = mpSearch.getPath();
        //     cv::Mat temp_img = my_search_problem->image_to_display.clone();
        //     // furthest_grandparent->plotNode({255, 102, 102));
        //     // n.furthest_grandparent->plotNode({102, 102, 255));
        //     // furthest_grandparent->reconstructPathToNode();
        //     // n.furthest_grandparent->reconstructPathToNode();
        //     this_root->plotNode({255, 102, 102});
        //     n_root->plotNode({102, 102, 255});
        //     this_root->reconstructPathToNode();
        //     n_root->reconstructPathToNode();
        //     // plotPath(path, false);
        //     my_search_problem->image_to_display = temp_img;
        // }
    }
}

void myNode::generateMergePointRegion()
{
    findFurthestGrandParent();
    neighborhoodSearchProblem<myNode> mp_region_search(furthest_grandparent, R_MP_REGION_RADIUS, 0, 0, &my_search_problem->startNode, my_search_problem->image_to_display, my_search_problem->occupancyMap);
    mp_region_search.search();

    auto mp_region = mp_region_search.collectNeighborhood();
    for (auto &n : mp_region)
    {
        n->isMergePoint = true;
        n->plotMergePoint();
    }
}

int myNode::getNumberOfCopies() const
{
    myNode n(x, y, z);
    std::vector<myNode *> nodes_at_same_xy = my_search_problem->all_nodes_set_p->getall(n, compare_by_coord_only);
    return nodes_at_same_xy.size();
}

// Main function for constructing and running the search problem and generating the results
int main(int argc, char *argv[])
{
    // RUNTIME_VERBOSE_SWITCH = 0;
    compute_program_path();

    std::string expt_f_name = program_path + "../expt/3d_envs.json", expt_name = "m1";
    std::string param_f_name = program_path + "../expt/params.json";
    std::string param_setName = "3d_regular";
    if (argc == 2)
    {
        expt_name = argv[1];
    }
    else if (argc > 2)
    {
        expt_name = argv[1];
        param_setName = argv[2];
    }

    printf(_BOLD _YELLOW "Note: " YELLOW_ BOLD_ "Using algorithm " _YELLOW MAKESTR(_DOSL_ALGORITHM) YELLOW_
           ". Run 'make' to recompile with a different algorithm.\n"
           "Note: This program currently does not support 'ThetaStar' algorithm.\n");

    searchProblem test_search_problem(expt_f_name, expt_name, param_f_name, param_setName);
    my_search_problem = &test_search_problem;
    test_search_problem.start_time = std::chrono::steady_clock::now();
    test_search_problem.search();
    test_search_problem.end_time = std::chrono::steady_clock::now();
    std::cout << "Computation time: " << std::chrono::duration_cast<std::chrono::seconds>(test_search_problem.end_time - test_search_problem.start_time).count() << "s" << std::endl;
#if VIS
    test_search_problem.my_fig.get_key();
    for (int i{test_search_problem.MIN_X}; i < test_search_problem.MAX_X + 1; i++)
    {
        for (int j{test_search_problem.MIN_Y}; j < test_search_problem.MAX_Y + 1; j++)
        {
            for (int k{test_search_problem.MIN_Z}; k < test_search_problem.MAX_Z + 1; k++)
            {
                auto color_at_box = test_search_problem.displayBoxes.at(i).at(j).at(k)->color();
                test_search_problem.displayBoxes.at(i).at(j).at(k)->visible() = (color_at_box == sglMake3vec(1, 0.64706, 0));
                // if (test_search_problem.displayBoxes.at(i).at(j).at(k)->visible())
                //     color_at_box = sglMake3vec(1, 0.64706, 0);
            }
        }
    }
    test_search_problem.my_fig.flush();
    int valid_paths = 0;
    std::vector<std::vector<myNode>> valid_path_vec;
    std::vector<double> valid_path_costs;
    std::vector<std::vector<myNode>> invalid_path_vec;
    std::vector<double> invalid_path_costs;
    for (int i = 0; i < test_search_problem.homotopyGoals.size(); ++i)
    {
        // get path
        auto path = test_search_problem.reconstruct_weighted_pointer_path(test_search_problem.homotopyGoals[i]);
        double cost = 0.0;
        bool containsMergePoint = false;

        // printf("Path size: %d", path.size());
        myNode thisPt = test_search_problem.startNode, lastPt;
        std::vector<myNode> allPts;
        std::vector<myNode> path_vec;
        for (int a = path.size() - 1; a >= 0; --a)
        {
            lastPt = thisPt;
            thisPt = myNode(0.0, 0.0, 0.0);
            for (auto it = path[a].begin(); it != path[a].end(); ++it)
            {
                if (it->first->isMergePoint)
                {
                    std::cout << "Path " << i << " contains a merge point." << std::endl;
                    containsMergePoint = true;
                    // break;
                }
                thisPt.x += it->second * it->first->x;
                thisPt.y += it->second * it->first->y;
                thisPt.z += it->second * it->first->z;
            }
            // if (containsMergePoint)
            //     break;
            path_vec.push_back(thisPt);
            allPts.push_back(thisPt);
            // printf("[%f,%f,%f]; ", (double)thisPt.x, (double)thisPt.y, (double)thisPt.z);
            // sglLine* line = test_search_problem.my_fig.addChild(sglLine(lastPt.x,lastPt.y,lastPt.z,thisPt.x,thisPt.y,thisPt.z));
            // sglLine *line = test_search_problem.my_fig.addChild(sglLine(lastPt.x, lastPt.y, lastPt.z, thisPt.x, thisPt.y, thisPt.z));
            // line->linewidth() = (i + 1) * 3;
            // test_search_problem.my_fig.addChild(sglSphere(thisPt.x, thisPt.y, thisPt.z, 0.2, 0, 0, 1));
            // // line->color() = sglMake3vec(1 - i / 2, 1, i / 2);
            // cost += sqrt((thisPt.x - lastPt.x) * (thisPt.x - lastPt.x) + (thisPt.y - lastPt.y) * (thisPt.y - lastPt.y) + (thisPt.z - lastPt.z) * (thisPt.z - lastPt.z));
        }
        if (containsMergePoint)
        {
            invalid_path_vec.push_back(path_vec);
            invalid_path_costs.push_back(test_search_problem.homotopyGoals[i].g_score);

            // continue;
        }
        else
        {
            valid_paths++;
            valid_path_vec.push_back(path_vec);
            valid_path_costs.push_back(test_search_problem.homotopyGoals[i].g_score);

            // myNode p2 = test_search_problem.startNode, p1;
            // for (int a = 1; a < path_vec.size(); a++)
            // {
            //     p1 = p2;
            //     p2 = path_vec[a];
            //     sglLine *line = test_search_problem.my_fig.addChild(sglLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z));
            //     line->linewidth() = 30;
            //     line->color() = sglMake3vec(1, 0, 0);
            //     sglSphere *sph = my_search_ptr->my_fig.addChild(sglSphere(p2.x, p2.y, p2.z, 0.1, 0.5, 0.5, 0.5));
            //     my_search_ptr->my_fig.flush();
            //     cost += sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
            // }
        }

        // test_search_problem.my_fig.flush();
        // test_search_problem.my_fig.get_key();

        // printf("\tclass: %d, cost: %f.\n", i, cost);
    }

    std::cout << ((valid_paths == my_search_problem->homotopyGoals.size()) ? "" : "Only ") << valid_paths << " valid path"
              << ((valid_paths != 1) ? "s " : " ")
              << "found out of "
              << my_search_problem->nClassesToFind << " desired path"
              << ((my_search_problem->nClassesToFind != 1) ? "s." : ".") << std::endl;

    for (int i{0}; i < valid_path_vec.size(); i++)
    {
        double cost = 0.0;
        myNode p2 = test_search_problem.startNode, p1;
        for (int a = 1; a < valid_path_vec[i].size(); a++)
        {
            p1 = p2;
            p2 = valid_path_vec[i][a];
            sglLine *line = test_search_problem.my_fig.addChild(sglLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z));
            line->linewidth() = 30;
            line->color() = sglMake3vec(0, 0.4, 0);
            sglSphere *sph = my_search_problem->my_fig.addChild(sglSphere(p2.x, p2.y, p2.z, 0.1, 0.5, 0.5, 0.5));
            my_search_problem->my_fig.flush();
            cost += sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
        }
        printf("\tValid path %d, cost: %f.\n", i, cost);
        std::cout << "Goal g-score (accounts for nonuniform path cost): " << valid_path_costs[i] << std::endl;

        test_search_problem.my_fig.flush();
        test_search_problem.my_fig.get_key();
    }

    for (int i{0}; i < invalid_path_vec.size(); i++)
    {
        double cost = 0.0;
        myNode p2 = test_search_problem.startNode, p1;
        for (int a = 1; a < invalid_path_vec[i].size(); a++)
        {
            p1 = p2;
            p2 = invalid_path_vec[i][a];
            sglLine *line = test_search_problem.my_fig.addChild(sglLine(p1.x, p1.y, p1.z, p2.x, p2.y, p2.z));
            line->linewidth() = 30;
            line->color() = sglMake3vec(0.8, 0.0, 0.8);
            sglSphere *sph = my_search_problem->my_fig.addChild(sglSphere(p2.x, p2.y, p2.z, 0.1, 0.5, 0.5, 0.5));
            my_search_problem->my_fig.flush();
            cost += sqrt((p2.x - p1.x) * (p2.x - p1.x) + (p2.y - p1.y) * (p2.y - p1.y) + (p2.z - p1.z) * (p2.z - p1.z));
        }
        printf("\tInvalid path %d, cost: %f.\n", i, cost);
        std::cout << "Goal g-score (accounts for nonuniform path cost): " << invalid_path_costs[i] << std::endl;

        test_search_problem.my_fig.flush();
        test_search_problem.my_fig.get_key();
    }

    // test_search_problem.my_fig.flush();
    test_search_problem.my_fig.flush();
    _dosl_cout << "Press any key to quit..." << _dosl_endl;
    // test_search_problem.my_fig.get_key();
    test_search_problem.my_fig.get_key();
    // test_search_problem.my_fig.close();
    // test_search_problem.my_fig.glutMainLoopThread.join();
#endif
    test_search_problem.clear();
    return 0;
}