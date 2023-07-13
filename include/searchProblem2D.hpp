// Header file for the searchProblem2D class

// Guards
#ifndef _SEARCHPROBLEM2D_H
#define _SEARCHPROBLEM2D_H

// Including libraries for parsing json files and maps
#include <dosl/aux-utils/cvParseMap2d.hpp>
#include "RSJparser.tcc"

// Planning related
#include "myNode2D.hpp"

// Global variables (algorithm parameters)
double R_HEURISTIC_WEIGHT;
int R_ROLLBACK_RADIUS;
double R_NEIGHBORHOOD_RADIUS;
int R_NEIGHBORHOOD_SEARCH_DEPTH;
bool R_CUT_POINT_CHECK;
double R_CP_LOWER_THRESHOLD;
double R_CP_UPPER_THRESHOLD;
double R_OVERLAP_FOR_CP;
double R_CP_PATH_PORTION;
double R_CP_GSCORE_DIFF;
double R_CPR_RADIUS;

class CompareByCoordOnly
{ // functor for invoking isCoordsEqual (required for getting all nodes at a coordinate)
public:
    bool operator()(const myNode2D &n1, const myNode2D &n2) { return n1.isCoordsEqual(n2); }
} compare_by_coord_only;

class searchProblem2D : public _DOSL_ALGORITHM::Algorithm<searchProblem2D, myNode2D, double>
{
public:
    // Fime names and JSON containers
    std::string map_image_fName, plot_image_fName, expt_fName, expt_folderName, expt_Name, param_fName;
    RSJresource expt_container, param_container;

    // Environment variables
    cvParseMap2d parsedMap;                     // used for obstacle checks
    COORD_TYPE MAX_X, MIN_X, MAX_Y, MIN_Y;      // boundaries of the map
    int nPathsToFind;                           // desired number of paths
    bool nonuniform;                            // cost type
    double NONUNIFORM_COST_MULTIPLIER;          // cost multiplier (CM)
    myNode2D startNode, goalNode, lastExpanded; //

    // Visualization
    cv::Mat originalMapMatrix; // pixel matrix of the original map
    cv::Mat image_to_display;  // display image (showing search progress)
    cv::Mat cleanMap;          // clean version of the pixel map
    double PLOT_SCALE;         // scale read from json file (effects how large the display window looks)
    double LINE_THICKNESS;     // thickness of the paths drawn

    int VIS_INTERVAL = 1000;    // update frequency of display image (in # expanded nodes)
    int REPORT_INTERVAL = 1000; // search progress report interval (in # expanded nodes)

    // Path progress
    int nPathsFound;
    std::vector<myNode2D> foundGoals;

    // Timing
    std::chrono::_V2::steady_clock::time_point start_time;
    std::chrono::_V2::steady_clock::time_point current_time;
    std::chrono::_V2::steady_clock::time_point end_time;

    // -----------------------------------------------------------

    // coordinate transformation (original pixels to display image)
    template <typename T>
    CvPoint cv_plot_coord(T x, T y)
    {
        return (cvPoint(round(PLOT_SCALE * (x - MIN_X)), round(PLOT_SCALE * (y - MIN_Y))));
    }

    // Plotting functions
    void cvPlotPoint(cv::Mat &img, CvPoint pt, CvScalar colr, int size = 1)
    {
        for (int i = pt.x; i <= pt.x + (size - 1); i++)
            for (int j = pt.y; j <= pt.y + (size - 1); j++)
            {
                if (i < 0 || i >= image_to_display.cols || j < 0 || j >= image_to_display.rows)
                    continue;
                img.at<cv::Vec3b>(j, i) = cv::Vec3b((uchar)colr.val[0], (uchar)colr.val[1], (uchar)colr.val[2]);
            }
    }

    void plotNode(cv::Mat &img, myNode2D n, CvScalar col)
    {
        cvPlotPoint(img, cv_plot_coord(n.x, n.y), col, PLOT_SCALE);
    }

    void plotCutPoint(myNode2D n)
    {
        plotNode(image_to_display, n, cvScalar(0, 165, 255));
        plotNode(cleanMap, n, cvScalar(0, 165, 255));
    }

    void cvPlotStartGoal(cv::Mat &img)
    {
        // Plot start
        cv::Point start = cv_plot_coord(startNode.x, startNode.y);
        cv::Rect rect_stroke(start.x - 25, start.y - 25, 50, 50);
        cv::Rect rect(start.x - 20, start.y - 20, 40, 40);
        cv::rectangle(img, rect_stroke, cvScalar(0, 0, 0), -1);
        cv::rectangle(img, rect, cvScalar(255, 255, 0), -1);
        cv::Point text_start(start.x - 10, start.y + 10);
        cv::putText(img, "S", text_start, cv::FONT_HERSHEY_SIMPLEX,
                    1, cvScalar(0, 0, 0), 2, cv::LINE_AA);
        // Plot goal
        cv::Point goal = cv_plot_coord(goalNode.x, goalNode.y);
        cv::Point pts_stroke[1][3];
        pts_stroke[0][0] = cv::Point(goal.x, goal.y - 45);
        pts_stroke[0][1] = cv::Point(goal.x - 37.5, goal.y + 28);
        pts_stroke[0][2] = cv::Point(goal.x + 37.5, goal.y + 28);
        const cv::Point *ppt_stroke[1] = {pts_stroke[0]};
        int npt[] = {3};
        cv::fillPoly(img, ppt_stroke, npt, 1, cvScalar(0, 0, 0), 1);
        cv::Point pts[1][3];
        pts[0][0] = cv::Point(goal.x, goal.y - 30);
        pts[0][1] = cv::Point(goal.x - 25, goal.y + 20);
        pts[0][2] = cv::Point(goal.x + 25, goal.y + 20);
        const cv::Point *ppt[1] = {pts[0]};
        cv::fillPoly(img, ppt, npt, 1, cvScalar(0, 255, 255), 1);
        cv::Point goal_text_start(goal.x - 10, goal.y + 10);
        cv::putText(img, "G", goal_text_start, cv::FONT_HERSHEY_SIMPLEX,
                    1, cvScalar(0, 0, 0), 2, cv::LINE_AA);
    }

    void popUpText(std::string text)
    {
        cv::Mat temp_img = image_to_display.clone();
        cv::Size textSize = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 1, 2, 0);
        cv::Point boxTopLeftCorner(image_to_display.size().width / 2 - textSize.width / 2 - 20,
                                   image_to_display.size().height / 2 - textSize.height / 2 - 20);
        cv::Point boxBottomRightCorner(image_to_display.size().width / 2 + textSize.width / 2 + 20,
                                       image_to_display.size().height / 2 + textSize.height / 2 + 20);
        cv::rectangle(image_to_display, boxTopLeftCorner, boxBottomRightCorner,
                      cvScalar(0, 0, 255), cv::FILLED, cv::LINE_AA);
        cv::putText(image_to_display, text,
                    cv::Point(image_to_display.size().width / 2 - textSize.width / 2,
                              image_to_display.size().height / 2 - textSize.height / 2 + 20),
                    cv::FONT_HERSHEY_SIMPLEX, 1, cvScalar(255, 255, 255), 2, cv::LINE_AA);
        cv::imshow("Display window", image_to_display);
        cv::waitKey(0);
        image_to_display = temp_img.clone();
    }

    void plotPath(cv::Mat &img, std::vector<std::unordered_map<myNode2D *, double>> &path, CvScalar col)
    {
        myNode2D thisPt = startNode, lastPt;
        for (int a = path.size() - 1; a >= 0; --a)
        {
            lastPt = thisPt;
            thisPt = myNode2D(0.0, 0.0);
            for (auto it = path[a].begin(); it != path[a].end(); ++it)
            {
                thisPt.x += it->second * it->first->x;
                thisPt.y += it->second * it->first->y;
            }
            cv::line(img,
                     cv_plot_coord(thisPt.x, thisPt.y), cv_plot_coord(lastPt.x, lastPt.y),
                     col,
                     LINE_THICKNESS * PLOT_SCALE * 0.5);
        }
    }

    // Constructor
    searchProblem2D(std::string expt_f_name, std::string expt_name, std::string param_f_name, std::string param_setName)
    {
        expt_fName = expt_f_name;
        expt_Name = expt_name;
        expt_folderName = expt_fName.substr(0, expt_fName.find_last_of("/\\") + 1);

        param_fName = param_f_name;

        // Read from file
        std::ifstream my_fstream(expt_fName);
        expt_container = RSJresource(my_fstream)[expt_Name];
        std::ifstream param_fstream(param_fName);
        param_container = RSJresource(param_fstream)[param_setName];

        // Read map
        map_image_fName = expt_folderName + expt_container["environment"]["pixmap"].as<std::string>();
        parsedMap = cvParseMap2d(map_image_fName, true);
        plot_image_fName = expt_folderName + expt_container["plot_options"]["plot_map"].as<std::string>();

        // Cost type
        if (expt_container["environment"]["cost_type"].exists())
        {
            if (expt_container["environment"]["cost_type"].as<std::string>() == "nonuniform")
            {
                nonuniform = true;
                originalMapMatrix = cv::imread(plot_image_fName, CV_LOAD_IMAGE_COLOR);
                NONUNIFORM_COST_MULTIPLIER = expt_container["environment"]["cost_multiplier"].as<double>(4.0);
            }
            else
            {
                nonuniform = false;
                originalMapMatrix = cv::imread(map_image_fName, CV_LOAD_IMAGE_COLOR);
            }
        }
        else
        {
            nonuniform = false;
            originalMapMatrix = cv::imread(map_image_fName, CV_LOAD_IMAGE_COLOR);
        }

        // plot options
        PLOT_SCALE = expt_container["plot_options"]["plot_scale"].as<double>(1.0);
        LINE_THICKNESS = expt_container["plot_options"]["line_thickness"].as<double>(2.0);

        // algorithm parameters
        R_HEURISTIC_WEIGHT = param_container["HEURISTIC_WEIGHT"].as<double>(1.0);
        R_ROLLBACK_RADIUS = param_container["ROLLBACK_RADIUS"].as<int>(1);
        R_NEIGHBORHOOD_RADIUS = param_container["NEIGHBORHOOD_RADIUS"].as<double>(1);
        R_NEIGHBORHOOD_SEARCH_DEPTH = param_container["NEIGHBORHOOD_SEARCH_DEPTH"].as<int>(1);
        R_CUT_POINT_CHECK = param_container["CUT_POINT_CHECK"].as<bool>(0);
        R_CP_LOWER_THRESHOLD = param_container["CP_LOWER_THRESHOLD"].as<double>(1.0);
        R_CP_UPPER_THRESHOLD = param_container["CP_UPPER_THRESHOLD"].as<double>(1.0);
        R_OVERLAP_FOR_CP = param_container["OVERLAP_FOR_CP"].as<double>(1.0);
        R_CP_PATH_PORTION = param_container["CP_PATH_PORTION"].as<double>(1.0);
        R_CP_GSCORE_DIFF = param_container["CP_GSCORE_DIFF"].as<double>(1.0);
        R_CPR_RADIUS = param_container["CPR_RADIUS"].as<double>(1.0);

        std::cout << "=============================================================" << std::endl;
        std::cout << "Running with algorithm parameter set: " << param_setName << std::endl;
        std::cout << "Heuristic weight: " << R_HEURISTIC_WEIGHT << std::endl;
        std::cout << "Rollback radius: " << R_ROLLBACK_RADIUS << std::endl;
        std::cout << "Neighborhood radius: " << R_NEIGHBORHOOD_RADIUS << std::endl;
        std::cout << "Neighborhood search depth: " << R_NEIGHBORHOOD_SEARCH_DEPTH << std::endl;
        std::cout << "Cut point check: " << R_CUT_POINT_CHECK << std::endl;
        if (R_CUT_POINT_CHECK)
        {
            std::cout << "Cut point overlap threshold: " << R_OVERLAP_FOR_CP << std::endl;
            std::cout << "Cut point lower threshold: " << R_CP_LOWER_THRESHOLD << std::endl;
            std::cout << "Cut point upper threshold: " << R_CP_UPPER_THRESHOLD << std::endl;
            std::cout << "Cut point path portion: " << R_CP_PATH_PORTION << std::endl;
            std::cout << "Cut point g-score diff: " << R_CP_GSCORE_DIFF << std::endl;
            std::cout << "Cut point region radius: " << R_CPR_RADIUS << std::endl;
        }

        // read data for planning
        MAX_X = parsedMap.width();
        MIN_X = 0;
        MAX_Y = parsedMap.height();
        MIN_Y = 0;
        startNode = myNode2D(expt_container["start"][0].as<int>(), expt_container["start"][1].as<int>());
        startNode.parent = &startNode; // assign itself as parent to the startNode
        goalNode = myNode2D(expt_container["goal"][0].as<int>(), expt_container["goal"][1].as<int>());
        nPathsToFind = expt_container["n_paths"].as<int>(2);
        nPathsFound = 0;

        std::cout << "=============================================================" << std::endl;
        std::cout << "Running with map parameters: " << expt_Name << std::endl;
        std::cout << "Nonuniform: " << nonuniform << std::endl;
        if (nonuniform)
            std::cout << "Nonuniform cost multiplier: " << NONUNIFORM_COST_MULTIPLIER << std::endl;
        std::cout << "Plot scale: " << PLOT_SCALE << std::endl;
        std::cout << "Line thickness: " << LINE_THICKNESS << std::endl;
        std::cout << "Number of paths to find: " << nPathsToFind << std::endl;
        startNode.print("Start Node: ");
        goalNode.print("Goal Node: ");
        std::cout << "=============================================================" << std::endl;

        // visualization
        image_to_display = originalMapMatrix.clone();
        cv::resize(image_to_display, image_to_display, cv::Size(), PLOT_SCALE, PLOT_SCALE);
        cleanMap = image_to_display.clone(); // copy resized version of original map, store as a clean copy
        cv::namedWindow("Display window", cv::WINDOW_AUTOSIZE);
        cv::moveWindow("Display window", 50, 50);
        popUpText("Press any key to show start and goal");
        cv::imshow("Display window", image_to_display);
        cvPlotStartGoal(image_to_display);
        cv::imshow("Display window", image_to_display);
        popUpText("Press any key to start the search");
        cvPlotStartGoal(cleanMap);

        // Set planner variables
        all_nodes_set_p->reserve(ceil(MAX_X - MIN_X + 1));
    }

    // -----------------------------------------------------------
    // in bounds check
    bool isNodeInWorkspace(const myNode2D &tn)
    {
        if (tn.x < MIN_X || tn.x >= MAX_X || tn.y < MIN_Y || tn.y >= MAX_Y)
            return (false);
        return (true);
    }

    // in bounds + obstacles check
    bool isEdgeAccessible(const myNode2D &tn1, const myNode2D &tn2)
    {
        if ((!isNodeInWorkspace(tn1)) || !(isNodeInWorkspace(tn2)))
            return (false);

        // the following works only for 8-connected grid!!
        if (tn1.x != tn2.x && tn1.y != tn2.y) // diagonal edge
            return (parsedMap.isFree(round(MIN(tn1.x, tn2.x)), round(MIN(tn1.y, tn2.y))));
        else if (tn1.x == tn2.x) // need to check two cells
            return (!((tn1.x == MAX_X || parsedMap.isObstacle(round(tn1.x), round(MIN(tn1.y, tn2.y)))) &
                      (tn1.x == MIN_X || parsedMap.isObstacle(round(tn1.x - 1), round(MIN(tn1.y, tn2.y))))));
        else if (tn1.y == tn2.y) // need to check two cells
            return (!((tn1.y == MAX_Y || parsedMap.isObstacle(round(MIN(tn1.x, tn2.x)), round(tn1.y))) &
                      (tn1.y == MIN_Y || parsedMap.isObstacle(round(MIN(tn1.x, tn2.x)), round(tn1.y - 1)))));

        return (true);
    }

    // Nonuniform path cost computation (refer to Section 3F - Cost multiplier analysis for details)
    double costAtPixel(const int x, const int y) const
    {
        double cost_multiplier{0};
        cv::Vec3b color = originalMapMatrix.at<cv::Vec3b>(cv::Point(x, y));
        cost_multiplier = 1.0 + NONUNIFORM_COST_MULTIPLIER * (255.0 - color.val[0]) / 255.0;
        return cost_multiplier;
    }

    double getCostMultiplier(const myNode2D &nTo, const myNode2D &nFrom) const
    {
        double m{0};
        m += costAtPixel(nTo.x, nTo.y);
        m += costAtPixel(nFrom.x, nFrom.y);
        return (m / 2);
    }

    // -----------------------------------------------------------

    void getSuccessors(myNode2D &n, std::vector<myNode2D> *s, std::vector<double> *c) // *** This must be defined
    {

        myNode2D tn; // dummy node instance
        tn.genNo = n.genNo + 1;

        tn.parent = &n; // assign parent pointer

        // loop through in all directions
        for (int a = -1; a <= 1; ++a)
            for (int b = -1; b <= 1; ++b)
            {
                if (a == 0 && b == 0)
                    continue;

// Do not create degenerate simplices
#ifdef DOSL_ALGORITHM_SStar
                int xParity = ((int)round(fabs(n.x))) % 2;
                if (xParity == 0 && (a != 0 && b == -1))
                    continue;
                if (xParity == 1 && (a != 0 && b == 1))
                    continue;
#endif

                // New coordinates
                tn.x = n.x + a;
                tn.y = n.y + b;

                if (!isEdgeAccessible(tn, n))
                    continue;

                double dx = tn.x - n.x, dy = tn.y - n.y;

                // Non uniform cost computation
                double cost_multiplier{1};
                if (nonuniform)
                {
                    cost_multiplier = getCostMultiplier(tn, n);
                }
                double computed_cost{cost_multiplier * sqrt(a * a + b * b)};

                s->push_back(tn);
                c->push_back(computed_cost);
            }
    }

    // Visibility check required for ThetaStar
    bool isSegmentFree(myNode2D &n1, myNode2D &n2, double *c)
    {
        double dx = (double)(n2.x - n1.x), dy = (double)(n2.y - n1.y);
        *c = sqrt(dx * dx + dy * dy);

        double step_count = 2.0 * ceil(*c);
        double xstep = dx / step_count, ystep = dy / step_count;

        double xx, yy;
        for (int a = 0; a < step_count; ++a)
        {
            xx = n1.x + a * xstep;
            yy = n1.y + a * ystep;
            if (parsedMap.isObstacle((int)round(xx), (int)round(yy)))
                return (false);
        }
        return (true);
    }

    // -----------------------------------------------------------

    double getHeuristics(myNode2D &n)
    {
        // With euclidean heuristic

        // double dx = goalNode.x - n.x;
        // double dy = goalNode.y - n.y;
        // return (sqrt(dx * dx + dy * dy));

        return (0.0); // Dijkstra's
    }

    // -----------------------------------------------------------

    std::vector<myNode2D> getStartNodes(void)
    {
        std::vector<myNode2D> startNodes;
        startNodes.push_back(startNode);
        return (startNodes);
    }

    // -----------------------------------------------------------

    void nodeEvent(myNode2D &n, unsigned int e)
    {
        CvScalar col = cvScalar(0.0, 0.0, 0.0);

        // --------------------------------------------
        if (e & EXPANDED)
        {
            lastExpanded = n;
            bool cameFromNull = false;
#ifdef DOSL_ALGORITHM_SStar
            cameFromNull = (n.CameFromSimplex == NULL);
#endif

            if (!cameFromNull)
            {
                // Determine node color for plotting
                std::vector<myNode2D *> nodes_at_same_xy = all_nodes_set_p->getall(n, compare_by_coord_only);

                double rb_intensity = MAX(0.0, 255.0 - 50.0 * nodes_at_same_xy.size());
                col = cvScalar(rb_intensity, 255.0, rb_intensity); // shades of green
                if (n.isCutPoint)
                    col = cvScalar(0, 165, 255);
            }
            if (expand_count % REPORT_INTERVAL == 0)
            {
                cv::imshow("Display window", image_to_display);
                current_time = std::chrono::steady_clock::now();
                std::cout << "Expanded " << expand_count << " vertices. Time since start: " << std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() << "s" << std::endl;
            }
        }

        else if ((e & HEAP) == PUSHED)
        {
            col = cvScalar(255.0, 0.0, 0.0); // blue
            if (n.genNo == 0)
                n.parent = &n; // reassign startNode parent (correct pointer this time)
        }

        else if (e & UNEXPANDED)
        {
            col = cvScalar(255.0, 0.0, 150.0); // purple
        }
        else if (e & UPDATED)
        {
            // To update the parent based on new came from info
            n.parent = n.oneStepRollback();
            n.getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_NEIGHBORHOOD_SEARCH_DEPTH);
            col = cvScalar(100.0, 100.0, 100.0);
        }

        else if (e & ERROR)
        {
            _dosl_cout << "\033[1;34;47m================================ nodeEvent (ERROR) start =================================\033[0m\n"
                       << _dosl_endl;
            col = cvScalar(120, 100, 200);
            plotNode(image_to_display, n, col);
            cv::imshow("Display window", image_to_display);
            cvWaitKey();
            current_time = std::chrono::steady_clock::now();
        }

        else
            return;

        //-------------------------------------------

        if (n.x < MAX_X && n.y < MAX_Y && parsedMap.isFree(round(n.x), round(n.y)))
            plotNode(image_to_display, n, col);

        if (n.isCutPoint)
            plotCutPoint(n);

        if (expand_count % VIS_INTERVAL == 0 || node_heap_p->size() == 0)
        {
            cvPlotStartGoal(image_to_display);
            std::cout << std::flush;
            cv::imshow("Display window", image_to_display);
            char key = (char)cv::waitKey(1); // explicit cast
        }
    }

    // ---------------------------------------
    // Stop conditions
    bool stopSearch(myNode2D &n)
    {
        if (n.isCoordsEqual(goalNode))
        {
            foundGoals.push_back(n);
            ++nPathsFound;
            n.print("Found a path to ");
            current_time = std::chrono::steady_clock::now();
            std::cout << "Found path " << nPathsFound << "/" << nPathsToFind << ". Expanded " << expand_count << " vertices. Time since start: " << std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() << "s" << std::endl;
            cvPlotStartGoal(image_to_display);
            if (nPathsFound >= nPathsToFind)
                return (true);
        }
        return (false);
    }
};

#endif