// Header file for the searchProblem3D class

// Guards
#ifndef _SEARCHPROBLEM3D_H
#define _SEARCHPROBLEM3D_H

// Required for RSJParser
#include <fstream>

// Including libraries for parsing json files and maps
#include "RSJparser.tcc"

// Planning related
#include "myNode3D.hpp" // myNode class based on DOSL

// For 3D visualization
#include "sgl/sgl"

#define bool bool

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
    bool operator()(const myNode3D &n1, const myNode3D &n2) { return n1.isCoordsEqual(n2); }
} compare_by_coord_only;

class searchProblem3D : public _DOSL_ALGORITHM::Algorithm<searchProblem3D, myNode3D, double>
{
public:
    // Fime names and JSON containers
    std::string map_image_fName, expt_fName, expt_folderName, expt_Name, param_fName;
    RSJresource expt_container, param_container;

    // Environment variables
    int env_scale;                                       // scaling of the environment (defined in container)
    COORD_TYPE MAX_X, MIN_X, MAX_Y, MIN_Y, MAX_Z, MIN_Z; // boundaries of the map
    std::vector<cv::Point3d> obstacleCorners;            // obstacle corners (defined in container)
    std::vector<cv::Point3d> obstacleDims;               // obstacle dimensions (defined in container)
    cv::Mat occupancyMap;                                // occupancy map to be used for obstacle check
    int nPathsToFind;                                    // desired number of paths
    myNode3D startNode, goalNode, lastExpanded;          //

    // Visualization
    sglFigure figure_to_display;                                  // sgl object for the 3D visualization
    std::vector<std::vector<std::vector<sglBox *>>> displayBoxes; // colored boxed to display search progress
    double LINE_THICKNESS;                                        // thickness of the paths drawn

    int REPORT_INTERVAL = 100; // search progress report interval (in # expanded nodes)

    // Path progress
    int nPathsFound;
    std::vector<myNode3D> foundGoals;

    // Timing
    std::chrono::_V2::steady_clock::time_point start_time;
    std::chrono::_V2::steady_clock::time_point current_time;
    std::chrono::_V2::steady_clock::time_point end_time;

    // Edges of the graph (designed to eliminate any degenerate simplices)
    double edges[14][3] = {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 1.0, 0.0}, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}, {1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {0.0, -1.0, -1.0}, {0.0, 1.0, 1.0}, {-1.0, 0.0, -1.0}, {1.0, 0.0, 1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}};

    // Plotting functions
    void plotNode(myNode3D n, std::vector<double> col)
    {
        displayBoxes.at(n.x).at(n.y).at(n.z)->color() = col;
        displayBoxes.at(n.x).at(n.y).at(n.z)->visible() = true;
    }

    void plotCutPoint(myNode3D n)
    {
        plotNode(n, {1, 0.6, 0});
        figure_to_display.flush();
        sleep(0.5);
    }

    void plotPath(std::vector<std::unordered_map<myNode3D *, double>> &path, std::vector<double> col)
    {
        myNode3D thisPt = startNode, lastPt;
        for (int a = path.size() - 1; a >= 0; --a)
        {
            lastPt = thisPt;
            thisPt = myNode3D(0.0, 0.0, 0.0);
            for (auto it = path[a].begin(); it != path[a].end(); ++it)
            {
                thisPt.x += it->second * it->first->x;
                thisPt.y += it->second * it->first->y;
                thisPt.z += it->second * it->first->z;
            }
            sglLine *line = figure_to_display.addChild(sglLine(lastPt.x, lastPt.y, lastPt.z, thisPt.x, thisPt.y, thisPt.z));
            line->linewidth() = LINE_THICKNESS * 30;
            line->color() = col;
            // sglSphere *sph = figure_to_display.addChild(sglSphere(p2.x, p2.y, p2.z, 0.1, 0.5, 0.5, 0.5));
        }
    }

    // Constructor
    searchProblem3D(std::string expt_f_name, std::string expt_name, std::string param_f_name, std::string param_setName)
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

        // Loading map
        if (expt_container["environment"].exists())
        {
            if (expt_container["environment"]["scale"].exists())
                env_scale = expt_container["environment"]["scale"].as<int>();
            else
                env_scale = 1;
            MAX_X = expt_container["environment"]["map_dims"][0].as<int>() * env_scale;
            MAX_Y = expt_container["environment"]["map_dims"][1].as<int>() * env_scale;
            MAX_Z = expt_container["environment"]["map_dims"][2].as<int>() * env_scale;
            MIN_X = 0;
            MIN_Y = 0;
            MIN_Z = 0;

            for (int i{MIN_X}; i < MAX_X + 1; i++)
            {
                std::vector<std::vector<sglBox *>> xBoxes;
                for (int j{MIN_Y}; j < MAX_Y + 1; j++)
                {
                    std::vector<sglBox *> yBoxes;
                    for (int k{MIN_Z}; k < MAX_Z + 1; k++)
                    {
                        sglBox *curr_box = figure_to_display.addChild(sglBox(i, j, k, i + 1, j + 1, k + 0.05, 1, 1, 1));
                        curr_box->visible() = false;
                        yBoxes.push_back(curr_box);
                    }
                    xBoxes.push_back(yBoxes);
                }
                displayBoxes.push_back(xBoxes);
            }

            int out[3] = {MAX_X, MAX_Y, MAX_Z};
            occupancyMap = cv::Mat(3, out, CV_32S, cv::Scalar(0));
            for (auto c : expt_container["environment"]["obstacle_corners"].as_array())
            {
                obstacleCorners.push_back(cv::Point3d(c[0].as<int>(), c[1].as<int>(), c[2].as<int>()) * env_scale);
            }
            for (auto d : expt_container["environment"]["obstacle_dims"].as_array())
            {
                obstacleDims.push_back(cv::Point3d(d[0].as<int>(), d[1].as<int>(), d[2].as<int>()) * env_scale);
            }
            if (obstacleCorners.size() != obstacleDims.size())
            {
                _dosl_cout << _BOLD _RED << "Obstacle corners and dims mismatch!" << RED_ BOLD_ << _dosl_endl;
                exit(0);
            }
        }
        else
        {
            _dosl_cout << _BOLD _RED << "Environment not specified!" << RED_ BOLD_ << _dosl_endl;
            exit(0);
        }
        _dosl_cout << _BOLD _GREEN << "Environment loaded!" << GREEN_ BOLD_ << _dosl_endl;
        for (int o{0}; o < obstacleCorners.size(); o++)
        {
            cv::Point3d corner = obstacleCorners[o];
            cv::Point3d dims = obstacleDims[o];
            for (int i{corner.x}; i < corner.x + dims.x; i++)
            {
                for (int j{corner.y}; j < corner.y + dims.y; j++)
                {
                    for (int k{corner.z}; k < corner.z + dims.z; k++)
                    {
                        occupancyMap.at<int>(i, j, k) = 1;
                    }
                }
            }
        }
        _dosl_cout << _BOLD _GREEN << "Occupancy Map generated!" << GREEN_ BOLD_ << _dosl_endl;

        // plot options
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
        startNode = myNode3D(expt_container["start"][0].as<int>(), expt_container["start"][1].as<int>(), expt_container["start"][2].as<int>()) * env_scale;
        startNode.parent = &startNode;
        goalNode = myNode3D(expt_container["goal"][0].as<int>(), expt_container["goal"][1].as<int>(), expt_container["goal"][2].as<int>()) * env_scale;
        nPathsToFind = expt_container["n_paths"].as<int>(2);
        nPathsFound = 0;

        std::cout << "=============================================================" << std::endl;
        std::cout << "Running with map parameters: " << expt_Name << std::endl;
        std::cout << "Environment scale: " << env_scale << std::endl;
        std::cout << "Line thickness: " << LINE_THICKNESS << std::endl;
        std::cout << "Number of paths to find: " << nPathsToFind << std::endl;
        startNode.print("Start Node: ");
        goalNode.print("Goal Node: ");
        std::cout << "=============================================================" << std::endl;

        // Visualization
        figure_to_display.height = 1200;
        figure_to_display.width = 1200;
        if (expt_container["init_cam"].exists())
        {
            std::cout << "Init cam matrix: " << std::endl;
            std::cout << "[";
            for (int i{0}; i < expt_container["init_cam"].as_array().size() - 1; i++)
            {
                std::cout << "[";
                auto cv = expt_container["init_cam"].as_array().at(i);
                for (int j{0}; j < cv.size() - 1; j++)
                {
                    double val = cv.as_array().at(j).as<double>();
                    figure_to_display.init_cam_config.push_back(val);
                    std::cout << val << ", ";
                }
                double val = cv.as_array().at(cv.size() - 1).as<double>();
                figure_to_display.init_cam_config.push_back(val);
                std::cout << val;
                std::cout << "]" << std::endl;
            }
            std::cout << "[";
            auto cv = expt_container["init_cam"].as_array().at(expt_container["init_cam"].as_array().size() - 1);
            for (int j{0}; j < cv.size() - 1; j++)
            {
                double val = cv.as_array().at(j).as<double>();
                figure_to_display.init_cam_config.push_back(val);
                std::cout << val << ", ";
            }
            double val = cv.as_array().at(cv.size() - 1).as<double>();
            figure_to_display.init_cam_config.push_back(val);
            std::cout << val;
            std::cout << "]";
            std::cout << "]" << std::endl;
            if (expt_container["cam_scale"].exists())
            {
                double cam_scale = expt_container["cam_scale"].as<double>();
                figure_to_display.cam_scale = cam_scale;
            }
        }

        // Environment boundaries and xyz axis
        figure_to_display.init();
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MAX_X, MIN_Y, MIN_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MAX_Y, MIN_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MIN_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MAX_X, MIN_Y, MIN_Z, MAX_X, MIN_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MAX_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MIN_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MAX_Z, MAX_X, MIN_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MAX_Z, MIN_X, MAX_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MAX_Y, MIN_Z, MIN_X, MAX_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MAX_Y, MIN_Z, MAX_X, MAX_Y, MIN_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MAX_X, MIN_Y, MAX_Z, MAX_X, MAX_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MAX_X, MAX_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MAX_Y, MAX_Z, MAX_X, MAX_Y, MAX_Z, 0, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MAX_X * 10, MIN_Y, MIN_Z, 1, 0, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MAX_Y * 10, MIN_Z, 0, 1, 0));
        figure_to_display.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MIN_Y, MAX_Z * 10, 0, 0, 1));
        for (int o{0}; o < obstacleCorners.size(); o++)
        {
            cv::Point3d corner = obstacleCorners[o];
            cv::Point3d dims = obstacleDims[o];
            sglBox bx(corner.x, corner.y, corner.z, corner.x + dims.x, corner.y + dims.y, corner.z + dims.z, 0.5, 0.5, 0.5, 0.9);
            figure_to_display.addChild(bx);
        }
        figure_to_display.flush();
        _dosl_cout << "Press any key to show start and goal" << _dosl_endl;
        figure_to_display.get_key();
        figure_to_display.addChild(sglSphere(startNode.x, startNode.y, startNode.z, 0.5, 1, 0, 0));
        figure_to_display.addChild(sglSphere(goalNode.x, goalNode.y, goalNode.z, 0.5, 0, 1, 0));
        figure_to_display.flush();
        _dosl_cout << "Press any key to start the search" << _dosl_endl;
        std::cout << "=============================================================" << std::endl;
        figure_to_display.get_key();

        // Set planner variables
        all_nodes_set_p->reserve(ceil(MAX_X - MIN_X + 1));
    }

    // -----------------------------------------------------------
    // in bounds check
    bool isNodeInWorkspace(const myNode3D &tn)
    {
        if (tn.x < MIN_X || tn.x > MAX_X || tn.y < MIN_Y || tn.y > MAX_Y || tn.z < MIN_Z || tn.z > MAX_Z)
            return (false);
        return (true);
    }

    // Obstacle check (get collision value)
    // Values depending on free space, at corner, on edge, on face, inside obstacle
    double getOccVal(const myNode3D &p)
    {
        if (!isNodeInWorkspace(p))
        {
            return 1;
        }
        double occVal = 0;
        for (int o{0}; o < obstacleCorners.size(); o++)
        {
            cv::Point3d corner = obstacleCorners[o];
            cv::Point3d dims = obstacleDims[o];
            // Corners 1/8
            if (p.x == corner.x && p.y == corner.y && p.z == corner.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x + dims.x && p.y == corner.y && p.z == corner.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x + dims.x && p.y == corner.y + dims.y && p.z == corner.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x && p.y == corner.y + dims.y && p.z == corner.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x && p.y == corner.y && p.z == corner.z + dims.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x + dims.x && p.y == corner.y && p.z == corner.z + dims.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x + dims.x && p.y == corner.y + dims.y && p.z == corner.z + dims.z)
                occVal += static_cast<double>(1) / 8;
            if (p.x == corner.x && p.y == corner.y + dims.y && p.z == corner.z + dims.z)
                occVal += static_cast<double>(1) / 8;
            // Edges 1/4
            // Bottom
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y == corner.y) && (p.z == corner.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x + dims.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z == corner.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y == corner.y + dims.y) && (p.z == corner.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z == corner.z))
                occVal += static_cast<double>(1) / 4;
            // Top
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y == corner.y) && (p.z == corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x + dims.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z == corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y == corner.y + dims.y) && (p.z == corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z == corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            // Sides
            if ((p.x == corner.x) && (p.y == corner.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x + dims.x) && (p.y == corner.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x + dims.x) && (p.y == corner.y + dims.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            if ((p.x == corner.x) && (p.y == corner.y + dims.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 4;
            // Faces 1/2
            // Bottom Face
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z == corner.z))
                occVal += static_cast<double>(1) / 2;
            // Top face
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z == corner.z + dims.z))
                occVal += static_cast<double>(1) / 2;
            // Front
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y == corner.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 2;
            // Right
            if ((p.x == corner.x + dims.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 2;
            // Back
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y == corner.y + dims.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 2;
            // Left
            if ((p.x == corner.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
                occVal += static_cast<double>(1) / 2;
            // Inside 1
            if ((p.x > corner.x) && (p.x < corner.x + dims.x) && (p.y > corner.y) && (p.y < corner.y + dims.y) && (p.z > corner.z) && (p.z < corner.z + dims.z))
            {
                return 1;
            }
        }
        // Environment Boundaries
        // Corners 1/8
        if (p.x == 0 && p.y == 0 && p.z == 0)
            occVal += static_cast<double>(7) / 8;
        if (p.x == MAX_X && p.y == 0 && p.z == 0)
            occVal += static_cast<double>(7) / 8;
        if (p.x == MAX_X && p.y == MAX_Y && p.z == 0)
            occVal += static_cast<double>(7) / 8;
        if (p.x == 0 && p.y == MAX_Y && p.z == 0)
            occVal += static_cast<double>(7) / 8;
        if (p.x == 0 && p.y == 0 && p.z == MAX_Z)
            occVal += static_cast<double>(7) / 8;
        if (p.x == MAX_X && p.y == 0 && p.z == MAX_Z)
            occVal += static_cast<double>(7) / 8;
        if (p.x == MAX_X && p.y == MAX_Y && p.z == MAX_Z)
            occVal += static_cast<double>(7) / 8;
        if (p.x == 0 && p.y == MAX_Y && p.z == MAX_Z)
            occVal += static_cast<double>(7) / 8;
        // Edges 1/4
        // Bottom
        // Edges 1/4
        // Bottom
        if ((p.x > 0) && (p.x < MAX_X) && (p.y == 0) && (p.z == 0))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == MAX_X) && (p.y > 0) && (p.y < MAX_Y) && (p.z == 0))
            occVal += static_cast<double>(3) / 4;
        if ((p.x > 0) && (p.x < MAX_X) && (p.y == MAX_Y) && (p.z == 0))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == 0) && (p.y > 0) && (p.y < MAX_Y) && (p.z == 0))
            occVal += static_cast<double>(3) / 4;
        // Top
        if ((p.x > 0) && (p.x < MAX_X) && (p.y == 0) && (p.z == MAX_Z))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == MAX_X) && (p.y > 0) && (p.y < MAX_Y) && (p.z == MAX_Z))
            occVal += static_cast<double>(3) / 4;
        if ((p.x > 0) && (p.x < MAX_X) && (p.y == MAX_Y) && (p.z == MAX_Z))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == 0) && (p.y > 0) && (p.y < MAX_Y) && (p.z == MAX_Z))
            occVal += static_cast<double>(3) / 4;
        // Sides
        if ((p.x == 0) && (p.y == 0) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == MAX_X) && (p.y == 0) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == MAX_X) && (p.y == MAX_Y) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(3) / 4;
        if ((p.x == 0) && (p.y == MAX_Y) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(3) / 4;
        // Faces 1/2
        // Bottom Face
        if ((p.x > 0) && (p.x < MAX_X) && (p.y > 0) && (p.y < MAX_Y) && (p.z == 0))
            occVal += static_cast<double>(1) / 2;
        // Top face
        if ((p.x > 0) && (p.x < MAX_X) && (p.y > 0) && (p.y < MAX_Y) && (p.z == MAX_Z))
            occVal += static_cast<double>(1) / 2;
        // Front
        if ((p.x > 0) && (p.x < MAX_X) && (p.y == 0) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(1) / 2;
        // Right
        if ((p.x == MAX_X) && (p.y > 0) && (p.y < MAX_Y) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(1) / 2;
        // Back
        if ((p.x > 0) && (p.x < MAX_X) && (p.y == MAX_Y) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(1) / 2;
        // Left
        if ((p.x == 0) && (p.y > 0) && (p.y < MAX_Y) && (p.z > 0) && (p.z < MAX_Z))
            occVal += static_cast<double>(1) / 2;
        return occVal;
    }

    bool areCoordsFree(const myNode3D &p)
    {
        return (getOccVal(p) != 1);
    }

    bool areCoordsFreeWithTolerance(const myNode3D &p)
    {
        for (int i{-1}; i <= 1; i++)
            for (int j{-1}; j <= 1; j++)
                for (int k{-1}; k <= 1; k++)
                    if (areCoordsFree(myNode3D(p.x + i * 0.001, p.y + j * 0.001, p.z + k * 0.001)))
                        return true;
        return false;
    }

    bool isEdgeAccessible(const myNode3D &tn1, const myNode3D &tn2)
    {
        if ((!isNodeInWorkspace(tn1)) || !(isNodeInWorkspace(tn2)))
            return (false);
        myNode3D mid_pt((tn1.x + tn2.x) / 2, (tn1.y + tn2.y) / 2, (tn1.z + tn2.z) / 2);
        return (areCoordsFreeWithTolerance(tn1) && areCoordsFreeWithTolerance(tn2) && areCoordsFreeWithTolerance(mid_pt));
    }

    // -----------------------------------------------------------

    void getSuccessors(myNode3D &n, std::vector<myNode3D> *s, std::vector<double> *c) // *** This must be defined
    {

        myNode3D tn; // dummy node instance
        tn.genNo = n.genNo + 1;

        tn.parent = &n; // assign parent pointer

        // loop through in all directions
        for (int a = 0; a < 14; ++a)
        {
            // New coordinates
            tn.x = n.x + edges[a][0];
            tn.y = n.y + edges[a][1];
            tn.z = n.z + edges[a][2];

            if (!isEdgeAccessible(tn, n))
                continue;

            double dx = tn.x - n.x, dy = tn.y - n.y, dz = tn.z - n.z;
            double computed_cost = sqrt(dx * dx + dy * dy + dz * dz);

            s->push_back(tn);
            c->push_back(computed_cost);
        }
    }

    // -----------------------------------------------------------
    double getHeuristics(myNode3D &n)
    {
        // With euclidean heuristic

        // double dx = goalNode.x - n.x;
        // double dy = goalNode.y - n.y;
        // double dz = goalNode.y - n.z;
        // return (sqrt(dx * dx + dy * dy + dz * dz));

        return (0.0); // Dijkstra's
    }

    // -----------------------------------------------------------
    std::vector<myNode3D> getStartNodes(void)
    {
        std::vector<myNode3D> startNodes;
        startNodes.push_back(startNode);
        return (startNodes);
    }

    // -----------------------------------------------------------
    void nodeEvent(myNode3D &n, unsigned int e)
    {
        std::vector<double> col{0, 0, 0};
        double alpha = 0.7;

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
                std::vector<myNode3D *> nodes_at_same_xy = all_nodes_set_p->getall(n, compare_by_coord_only);
                double rb_intensity = MAX(0.0, 1.0 - 0.2 * nodes_at_same_xy.size());
                col = {rb_intensity, 1.0, rb_intensity}; // shades of green
                if (n.isCutPoint)
                    plotCutPoint(n);
                else
                    plotNode(n, col);
                if (expand_count % REPORT_INTERVAL == 0)
                {
                    current_time = std::chrono::steady_clock::now();
                    std::cout << "Expanded " << expand_count << " vertices. Time since start: " << std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() << "s" << std::endl;
                }
            }
        }

        else if ((e & HEAP) == PUSHED)
        {
            col = {0, 0, 1};
            if (n.genNo == 0)
                n.parent = &n; // reassign startNode parent (correct pointer this time)
            plotNode(n, col);
        }
        else if (e & UPDATED)
        {
            // To update the parent based on new came from info
            n.parent = n.oneStepRollback();
            n.getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_NEIGHBORHOOD_SEARCH_DEPTH);
            plotNode(n, {0.5, 0.5, 0.5});
        }
        else if (e & ERROR)
        {
            _dosl_cout << "\033[1;34;47m================================ nodeEvent (ERROR) start =================================\033[0m\n"
                       << _dosl_endl;
            figure_to_display.flush();
            figure_to_display.get_key();
            current_time = std::chrono::steady_clock::now();
        }

        else
            return;
    }

    // ---------------------------------------
    // Stop conditions
    bool stopSearch(myNode3D &n)
    {
        if (n.isCoordsEqual(goalNode))
        {
            foundGoals.push_back(n);
            ++nPathsFound;
            n.print("Found a path to ");
            current_time = std::chrono::steady_clock::now();
            std::cout << "Found path " << nPathsFound << "/" << nPathsToFind << ". Expanded " << expand_count << " vertices. Time since start: " << std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() << "s" << std::endl;
            if (nPathsFound >= nPathsToFind)
            {
                figure_to_display.flush();
                figure_to_display.get_key();
                return (true);
            }
        }
        return (false);
    }
};

#endif