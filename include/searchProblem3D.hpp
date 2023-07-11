// Header file for the searchProblem class, for now class implementations are included
// Guards
#ifndef _SEARCHPROBLEM3D_H
#define _SEARCHPROBLEM3D_H

#include <fstream>
// Including libraries for parsing json files and maps
#include "RSJparser.tcc"
// User made
#include "myNode3D.hpp" // myNode class based on DOSL
#include "sgl/sgl"
#define bool bool

#define VIS 1

#define OVERLAP_VIS false

double R_EDGE_LENGTH;
double R_HEURISTIC_WEIGHT;
int R_ROLLBACK;
double R_NEIGHBORHOOD_RADIUS;
double R_NEIGHBORHOOD_INNER_RADIUS;
int R_MIN_NB_GENERATIONS;
int R_LINEAGE_DATA_GENERATION_THRESHOLD;
double R_NEIGHBORHOOD_OVERLAP_THRESHOLD;
int R_NEIGHBORHOOD_SIZE_THRESHOLD;
bool R_MP_CHECK;
double R_MP_LOWER_THRESHOLD;
double R_MP_UPPER_THRESHOLD;
double R_OVERLAP_FOR_MP;
double R_MP_PATH_LENGTH;
double R_MP_GSCORE_DIFF;
double R_MP_REGION_RADIUS;
bool R_PLOT_NEIGHBORHOOD;
bool R_PLOT_MP_NB;

int VIS_PAUSE = 0;    // cv::waitKey argument (useful for slowing and speeding the plot)
int VIS_INTERVAL = 1; // used for speeding and slowing display/search
double PLOT_SCALE;    // scale read from json file (effects how large the display window looks)
// COORD_TYPE MAX_X, MIN_X, MAX_Y, MIN_Y, MAX_Z, MIN_Z;

double edges[][3] = {{-1.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, {0.0, -1.0, 0.0}, {0.0, 1.0, 0.0}, {-1.0, -1.0, -1.0}, {1.0, 1.0, 1.0}, {1.0, -1.0, 0.0}, {-1.0, 1.0, 0.0}, {0.0, -1.0, -1.0}, {0.0, 1.0, 1.0}, {-1.0, 0.0, -1.0}, {1.0, 0.0, 1.0}, {0.0, 0.0, -1.0}, {0.0, 0.0, 1.0}};

class CompareByCoordOnly
{ // functor for invoking isCoordsEqual (required for getting all nodes at a coordinate)
public:
    bool operator()(const myNode &n1, const myNode &n2) { return n1.isCoordsEqual(n2); }
} compare_by_coord_only;

// Implementation of searchProblem class
class searchProblem : public _DOSL_ALGORITHM::Algorithm<searchProblem, myNode, double>
{
public:
    // Fime names and JSON objects
    std::string map_image_fName, plot_image_fName, expt_fName, expt_folderName, expt_Name, param_fName;
    RSJresource expt_container, param_container;
    int nClassesToFind;

    // Image display variables / parameters
    COORD_TYPE MAX_X, MIN_X, MAX_Y, MIN_Y, MAX_Z, MIN_Z;
    int env_scale;
    std::vector<cv::Point3d> obstacleCorners;
    std::vector<cv::Point3d> obstacleDims;
    cv::Mat occupancyMap;
    std::vector<std::vector<std::vector<sglBox *>>> displayBoxes;
    std::vector<std::vector<std::vector<sglBox *>>> overlapBoxes;
    std::vector<std::vector<std::vector<bool>>> visibleBoxes;
    myNode startNode, goalNode, lastExpanded;

    // homotopy classes
    int nClasses;
    std::vector<myNode> homotopyGoals;

    std::chrono::_V2::steady_clock::time_point start_time;
    std::chrono::_V2::steady_clock::time_point current_time;
    std::chrono::_V2::steady_clock::time_point end_time;

    // visuals
    sglFigure my_fig;
    int highest_visible_layer = 0;
    bool clr;
    cv::Mat image_to_display; // dummy for 2D compatibility

    // -----------------------------------------------------------
    // Constructor
    searchProblem(std::string expt_f_name, std::string expt_name, std::string param_f_name, std::string param_setName)
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
        clr = true;

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
#if VIS
            for (int i{MIN_X}; i < MAX_X + 1; i++)
            {
                std::vector<std::vector<sglBox *>> xBoxes;
                for (int j{MIN_Y}; j < MAX_Y + 1; j++)
                {
                    std::vector<sglBox *> yBoxes;
                    for (int k{MIN_Z}; k < MAX_Z + 1; k++)
                    {
                        sglBox *curr_box = my_fig.addChild(sglBox(i, j, k, i + 1, j + 1, k + 0.05, 1, 1, 1));
                        curr_box->visible() = false;
                        yBoxes.push_back(curr_box);
                    }
                    xBoxes.push_back(yBoxes);
                }
                displayBoxes.push_back(xBoxes);
            }
#endif
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
        // int ctct{0};
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
                        // ctct++;
                    }
                }
            }
        }
        // _dosl_cout << "ctct1: " << ctct << _dosl_endl;
        _dosl_cout << _BOLD _GREEN << "Occupancy Map generated!" << GREEN_ BOLD_ << _dosl_endl;

        // read data for planning
        startNode = myNode(expt_container["start"][0].as<int>(), expt_container["start"][1].as<int>(), expt_container["start"][2].as<int>()) * env_scale;
        startNode.print("Start Node: ");
        startNode.parent = &startNode;
        goalNode = myNode(expt_container["goal"][0].as<int>(), expt_container["goal"][1].as<int>(), expt_container["goal"][2].as<int>()) * env_scale;

#if REGULAR_SEARCH
        nClassesToFind = 1;
#else
        nClassesToFind = expt_container["top_class"].as<int>(2);
#endif
        nClasses = 0;

        R_EDGE_LENGTH = param_container["EDGE_LENGTH"].as<double>();
        R_HEURISTIC_WEIGHT = param_container["HEURISTIC_WEIGHT"].as<double>();
        R_ROLLBACK = param_container["ROLLBACK"].as<int>();
        R_NEIGHBORHOOD_RADIUS = param_container["NEIGHBORHOOD_RADIUS"].as<double>();
        R_NEIGHBORHOOD_INNER_RADIUS = param_container["NEIGHBORHOOD_INNER_RADIUS"].as<double>();
        R_MIN_NB_GENERATIONS = param_container["MIN_NB_GENERATIONS"].as<int>();
        R_LINEAGE_DATA_GENERATION_THRESHOLD = param_container["LINEAGE_DATA_GENERATION_THRESHOLD"].as<int>();
        R_NEIGHBORHOOD_OVERLAP_THRESHOLD = param_container["NEIGHBORHOOD_OVERLAP_THRESHOLD"].as<double>();
        R_NEIGHBORHOOD_SIZE_THRESHOLD = param_container["NEIGHBORHOOD_SIZE_THRESHOLD"].as<int>();
        R_MP_CHECK = param_container["MP_CHECK"].as<bool>();
        R_MP_LOWER_THRESHOLD = param_container["MP_LOWER_THRESHOLD"].as<double>();
        R_MP_UPPER_THRESHOLD = param_container["MP_UPPER_THRESHOLD"].as<double>();
        R_OVERLAP_FOR_MP = param_container["OVERLAP_FOR_MP"].as<double>();
        R_MP_PATH_LENGTH = param_container["MP_PATH_LENGTH"].as<double>();
        R_MP_GSCORE_DIFF = param_container["MP_GSCORE_DIFF"].as<double>();
        R_MP_REGION_RADIUS = param_container["MP_REGION_RADIUS"].as<double>();
        R_PLOT_NEIGHBORHOOD = param_container["plot_neighborhood"].as<bool>();
        R_PLOT_MP_NB = param_container["plot_mergepoint_nb"].as<bool>();

#if VIS
        my_fig.height = 1200;
        my_fig.width = 1200;
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
                    my_fig.init_cam_config.push_back(val);
                    std::cout << val << ", ";
                }
                double val = cv.as_array().at(cv.size() - 1).as<double>();
                my_fig.init_cam_config.push_back(val);
                std::cout << val;
                std::cout << "]" << std::endl;
            }
            std::cout << "[";
            auto cv = expt_container["init_cam"].as_array().at(expt_container["init_cam"].as_array().size() - 1);
            for (int j{0}; j < cv.size() - 1; j++)
            {
                double val = cv.as_array().at(j).as<double>();
                my_fig.init_cam_config.push_back(val);
                std::cout << val << ", ";
            }
            double val = cv.as_array().at(cv.size() - 1).as<double>();
            my_fig.init_cam_config.push_back(val);
            std::cout << val;
            std::cout << "]";
            std::cout << "]" << std::endl;
            if (expt_container["cam_scale"].exists())
            {
                double cam_scale = expt_container["cam_scale"].as<double>();
                my_fig.cam_scale = cam_scale;
            }
        }
        my_fig.init();

        my_fig.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MAX_X, MIN_Y, MIN_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MAX_Y, MIN_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MIN_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MAX_X, MIN_Y, MIN_Z, MAX_X, MIN_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MAX_X, MIN_Y, MIN_Z, MAX_X, MAX_Y, MIN_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MIN_Y, MAX_Z, MAX_X, MIN_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MIN_Y, MAX_Z, MIN_X, MAX_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MAX_Y, MIN_Z, MIN_X, MAX_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MAX_Y, MIN_Z, MAX_X, MAX_Y, MIN_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MAX_X, MIN_Y, MAX_Z, MAX_X, MAX_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MAX_X, MAX_Y, MIN_Z, MAX_X, MAX_Y, MAX_Z, 0, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MAX_Y, MAX_Z, MAX_X, MAX_Y, MAX_Z, 0, 0, 0));

        my_fig.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MAX_X * 10, MIN_Y, MIN_Z, 1, 0, 0));
        my_fig.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MAX_Y * 10, MIN_Z, 0, 1, 0));
        my_fig.addChild(sglLine(MIN_X, MIN_Y, MIN_Z, MIN_X, MIN_Y, MAX_Z * 10, 0, 0, 1));

        for (int o{0}; o < obstacleCorners.size(); o++)
        {
            cv::Point3d corner = obstacleCorners[o];
            cv::Point3d dims = obstacleDims[o];
            sglBox bx(corner.x, corner.y, corner.z, corner.x + dims.x, corner.y + dims.y, corner.z + dims.z, 0.5, 0.5, 0.5, 0.9);
            my_fig.addChild(bx);
        }
        my_fig.flush();
        _dosl_cout << "Press any key to start..." << _dosl_endl;
        my_fig.get_key();
#endif
        // Set planner variables
        all_nodes_set_p->reserve(ceil(MAX_X - MIN_X + 1));
    }

    // -----------------------------------------------------------

    // in bounds check
    bool isNodeInWorkspace(const myNode &tn)
    {
        if (tn.x < MIN_X || tn.x > MAX_X || tn.y < MIN_Y || tn.y > MAX_Y || tn.z < MIN_Z || tn.z > MAX_Z)
            return (false);
        return (true);
    }

    double getOccVal(const myNode &p, bool debug_mode = false)
    {
        if (!isNodeInWorkspace(p))
        {
            if (debug_mode)
                std::cout << "Not in workspace" << std::endl;
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
                if (debug_mode)
                {
                    std::cout << "p.x > corner.x: " << (p.x > corner.x) << std::endl;
                    std::cout << "p.x < corner.x + dims.x: " << (p.x < corner.x + dims.x) << std::endl;
                    std::cout << "p.y > corner.y: " << (p.y > corner.y) << std::endl;
                    std::cout << "p.y < corner.y + dims.y: " << (p.y < corner.y + dims.y) << std::endl;
                    std::cout << "p.z > corner.z: " << (p.z > corner.z) << std::endl;
                    std::cout << "p.z < corner.z + dims.z: " << (p.z < corner.z + dims.z) << std::endl;
                    std::cout << "Inside obstacle" << std::endl;
                }
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
        if (debug_mode)
            std::cout << "occVal: " << occVal << std::endl;
        return occVal;
    }

    bool areCoordsFree(const myNode &p, bool debug_mode = false)
    {
        return (getOccVal(p, debug_mode) != 1);
    }

    bool areCoordsFreeWithTolerance(const myNode &p)
    {
        for (int i{-1}; i <= 1; i++)
            for (int j{-1}; j <= 1; j++)
                for (int k{-1}; k <= 1; k++)
                    if (areCoordsFree(myNode(p.x + i * EPS, p.y + j * EPS, p.z + k * EPS)))
                        return true;
        return false;
    }

    bool isEdgeAccessibleNew(const myNode &tn1, const myNode &tn2, bool debug_mode = false)
    {
        if ((!isNodeInWorkspace(tn1)) || !(isNodeInWorkspace(tn2)))
            return (false);
        myNode mid_pt((tn1.x + tn2.x) / 2, (tn1.y + tn2.y) / 2, (tn1.z + tn2.z) / 2);
        if (debug_mode)
        {
            tn1.print("tn1: ");
            mid_pt.print("mid pt: ");
            tn2.print("tn2: ");
            std::cout << "tn1 free: " << areCoordsFree(tn1, true) << std::endl;
            std::cout << "mid free: " << areCoordsFree(mid_pt, true) << std::endl;
            std::cout << "tn2 free: " << areCoordsFree(tn2, true) << std::endl;
            std::cout << "tn1 free with tol: " << areCoordsFreeWithTolerance(tn1) << std::endl;
            std::cout << "mid free with tol: " << areCoordsFreeWithTolerance(mid_pt) << std::endl;
            std::cout << "tn2 free with tol: " << areCoordsFreeWithTolerance(tn2) << std::endl;
        }
        // return (areCoordsFree(tn1) && areCoordsFree(tn2) && areCoordsFree(mid_pt));
        return (areCoordsFreeWithTolerance(tn1) && areCoordsFreeWithTolerance(tn2) && areCoordsFreeWithTolerance(mid_pt));
    }

    bool isEdgeAccessibleAlternative(const myNode &tn1, const myNode &tn2)
    {
        if ((!isNodeInWorkspace(tn1)) || !(isNodeInWorkspace(tn2)))
            return (false);

        if (tn1.x == tn2.x)
        {
            if (tn1.y != tn2.y && tn1.z != tn2.z)
            { // diagonal edge
                if (!occupancyMap.at<int>(tn1.x, round(MIN(tn1.y, tn2.y)), round(MIN(tn1.z, tn2.z))))
                    return true;
                else
                {
                    if (tn1.x == MIN_X)
                        return false;
                    else if (!occupancyMap.at<int>(tn1.x - 1, round(MIN(tn1.y, tn2.y)), round(MIN(tn1.z, tn2.z))))
                        return true;
                    else
                        return false;
                }
            }
            else if (tn1.y == tn2.y)
            {
                if (!occupancyMap.at<int>(tn1.x, tn1.y, round(MIN(tn1.z, tn2.z))))
                    return true;
                else
                {
                    if (tn1.x == MIN_X && tn1.y == MIN_Y)
                        return false;
                    else if (tn1.x == MIN_X)
                    {
                        if (!occupancyMap.at<int>(tn1.x, tn1.y - 1, round(MIN(tn1.z, tn2.z))))
                            return true;
                        else
                            return false;
                    }
                    else if (tn1.y == MIN_Y)
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, tn1.y, round(MIN(tn1.z, tn2.z))))
                            return true;
                        else
                            return false;
                    }
                    else
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, tn1.y - 1, round(MIN(tn1.z, tn2.z))))
                            return true;
                        else
                            return false;
                    }
                }
            }
            else if (tn1.z == tn2.z)
            {
                if (!occupancyMap.at<int>(tn1.x, round(MIN(tn1.y, tn2.y)), tn1.z))
                    return true;
                else
                {
                    if (tn1.x == MIN_X && tn1.z == MIN_Z)
                        return false;
                    else if (tn1.x == MIN_X)
                    {
                        if (!occupancyMap.at<int>(tn1.x, round(MIN(tn1.y, tn2.y)), tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                    else if (tn1.z == MIN_Z)
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, round(MIN(tn1.y, tn2.y)), tn1.z))
                            return true;
                        else
                            return false;
                    }
                    else
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, round(MIN(tn1.y, tn2.y)), tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                }
            }
        }
        else if (tn1.y == tn2.y)
        {
            if (tn1.x != tn2.x && tn1.z != tn2.z)
            { // diagonal edge
                if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y, round(MIN(tn1.z, tn2.z))))
                    return true;
                else
                {
                    if (tn1.y == MIN_Y)
                        return false;
                    else if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y - 1, round(MIN(tn1.z, tn2.z))))
                        return true;
                    else
                        return false;
                }
            }
            else if (tn1.x == tn2.x)
            {
                if (!occupancyMap.at<int>(tn1.x, tn1.y, round(MIN(tn1.z, tn2.z))))
                    return true;
                else
                {
                    if (tn1.x == MIN_X && tn1.y == MIN_Y)
                        return false;
                    else if (tn1.x == MIN_X)
                    {
                        if (!occupancyMap.at<int>(tn1.x, tn1.y - 1, round(MIN(tn1.z, tn2.z))))
                            return true;
                        else
                            return false;
                    }
                    else if (tn1.y == MIN_Y)
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, tn1.y, round(MIN(tn1.z, tn2.z))))
                            return true;
                        else
                            return false;
                    }
                    else
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, tn1.y - 1, round(MIN(tn1.z, tn2.z))))
                            return true;
                        else
                            return false;
                    }
                }
            }
            else if (tn1.z == tn2.z)
            {
                if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y, tn1.z))
                    return true;
                else
                {
                    if (tn1.y == MIN_Y && tn1.z == MIN_Z)
                        return false;
                    else if (tn1.y == MIN_Y)
                    {
                        if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y, tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                    else if (tn1.z == MIN_Z)
                    {
                        if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y - 1, tn1.z))
                            return true;
                        else
                            return false;
                    }
                    else
                    {
                        if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y - 1, tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                }
            }
        }
        else if (tn1.z == tn2.z)
        {
            if (tn1.x != tn2.x && tn1.y != tn2.y)
            { // diagonal edge
                if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), round(MIN(tn1.y, tn2.y)), tn1.z))
                    return true;
                else
                {
                    if (tn1.z == MIN_Z)
                        return false;
                    else if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), round(MIN(tn1.y, tn2.y)), tn1.z - 1))
                        return true;
                    else
                        return false;
                }
            }
            else if (tn1.x == tn2.x)
            {
                if (!occupancyMap.at<int>(tn1.x, round(MIN(tn1.y, tn2.y)), tn1.z))
                    return true;
                else
                {
                    if (tn1.x == MIN_X && tn1.z == MIN_Z)
                        return false;
                    else if (tn1.x == MIN_X)
                    {
                        if (!occupancyMap.at<int>(tn1.x, round(MIN(tn1.y, tn2.y)), tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                    else if (tn1.z == MIN_Z)
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, round(MIN(tn1.y, tn2.y)), tn1.z))
                            return true;
                        else
                            return false;
                    }
                    else
                    {
                        if (!occupancyMap.at<int>(tn1.x - 1, round(MIN(tn1.y, tn2.y)), tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                }
            }
            else if (tn1.y == tn2.y)
            {
                if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y, tn1.z))
                    return true;
                else
                {
                    if (tn1.y == MIN_Y && tn1.z == MIN_Z)
                        return false;
                    else if (tn1.y == MIN_Y)
                    {
                        if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y, tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                    else if (tn1.z == MIN_Z)
                    {
                        if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y - 1, tn1.z))
                            return true;
                        else
                            return false;
                    }
                    else
                    {
                        if (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), tn1.y - 1, tn1.z - 1))
                            return true;
                        else
                            return false;
                    }
                }
            }
        }
        else
        {
            return (!occupancyMap.at<int>(round(MIN(tn1.x, tn2.x)), round(MIN(tn1.y, tn2.y)), round(MIN(tn1.z, tn2.z))));
        }

        return (true);
    }

    // -----------------------------------------------------------

    // void getSuccessors(myNode &n, std::vector<myNode> *s, std::vector<double> *c, myNode *grandparent) // *** This must be defined
    void getSuccessors(myNode &n, std::vector<myNode> *s, std::vector<double> *c) // *** This must be defined
    {

        // Update lineage data (via copy from parent), garbage otherwise
        n.lineage_data = n.parent->lineage_data;

        myNode tn; // dummy node instance
        tn.genNo = n.genNo + 1;

        // *AS: See if this one breaks
        // if (n.isMergePoint)
        // {
        //     // n.print("Alternative getSuccessors for mergePoint: ");
        //     for (auto it = n.parent_list.begin(); it != n.parent_list.end(); ++it)
        //     {
        //         s->push_back(*(it->first));
        //         c->push_back(it->second);
        //     }
        //     return;
        // }

        // Regular successor generation

        tn.parent = &n; // assign parent pointer
        // loop through in all directions

        for (int a = 0; a < 14; ++a)
        {
            tn.x = n.x + edges[a][0] * R_EDGE_LENGTH;
            tn.y = n.y + edges[a][1] * R_EDGE_LENGTH;
            tn.z = n.z + edges[a][2] * R_EDGE_LENGTH;

            // if (!isEdgeAccessible(tn, n))
            if (!isEdgeAccessibleNew(tn, n))
                continue;

            double dx = tn.x - n.x, dy = tn.y - n.y, dz = tn.z - n.z;
            double computed_cost = sqrt(dx * dx + dy * dy + dz * dz);
            tn.transition_cost = computed_cost;

            // Clear previous parents
            tn.parent_list.clear();
            // Insert new parent
            tn.parent_list.insert(std::make_pair(tn.parent, tn.transition_cost));

            s->push_back(tn);
            c->push_back(computed_cost);
        }
    }

    // -----------------------------------------------------------

    double getHeuristics(myNode &n)
    {
        // With euclidean heuristic

        // double dx = goalNode.x - n.x;
        // double dy = goalNode.y - n.y;
        // double dz = goalNode.y - n.z;
        // return (sqrt(dx * dx + dy * dy + dz * dz));

        return (0.0); // Dijkstra's
    }

    // -----------------------------------------------------------

    std::vector<myNode> getStartNodes(void)
    {
        std::vector<myNode> startNodes;
        startNodes.push_back(startNode);
#if VIS
        my_fig.addChild(sglSphere(startNode.x, startNode.y, startNode.z, 0.5, 1, 0, 0));
        // my_fig.addChild(sglSphere(4, 4, 16, 0.3, 0.2, 0.2, 0.2));
        my_fig.addChild(sglSphere(goalNode.x, goalNode.y, goalNode.z, 0.5, 0, 1, 0));
        my_fig.flush();
        my_fig.get_key();
#endif
        return (startNodes);
    }

    // -----------------------------------------------------------

    void nodeEvent(myNode &n, unsigned int e)
    {
        bool drawVertex = false;
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
                std::vector<myNode *> nodes_at_same_xy = all_nodes_set_p->getall(n, compare_by_coord_only);
                double rb_intensity = MAX(0.0, 1.0 - 0.2 * nodes_at_same_xy.size());
                col = {rb_intensity, 1.0, rb_intensity};
#if VIS
                if (n.isMergePoint)
                    displayBoxes.at(n.x).at(n.y).at(n.z)->color() = sglMake3vec(1, 0.64706, 0);
                else
                    displayBoxes.at(n.x).at(n.y).at(n.z)->color() = sglMake3vec(rb_intensity, 1.0, rb_intensity);

                displayBoxes.at(n.x).at(n.y).at(n.z)->visible() = true;
#endif
                if (expand_count % 1000 == 0)
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
            {
                n.parent = &n;
            }
#if VIS
            displayBoxes.at(n.x).at(n.y).at(n.z)->color() = sglMake3vec(0, 0, 1);
            displayBoxes.at(n.x).at(n.y).at(n.z)->visible() = true;
#endif
        }

        else if (e & UNEXPANDED)
        {
            // printf("Backtrack started!!\n");
        }
        else if (e & UPDATED)
        {
            n.parent = n.oneStepRollback();
            n.getNeighborhood(R_NEIGHBORHOOD_RADIUS, R_MIN_NB_GENERATIONS);
// n.getNeighborhood(n.NEIGHBORHOOD_RADIUS);
#if VIS
            displayBoxes.at(n.x).at(n.y).at(n.z)->color() = sglMake3vec(0.5, 0.5, 0.5);
            displayBoxes.at(n.x).at(n.y).at(n.z)->visible() = true;
#endif
        }
        else if (e & ERROR)
        {
            _dosl_cout << "\033[1;34;47m================================ nodeEvent (ERROR) start =================================\033[0m\n"
                       << _dosl_endl;
            n.print("isMergePoint=" + std::to_string(n.isMergePoint) + ", Error at this node: "); // nodeEvent with ERROR tag is called on this node
            n.printSuccessors();
#if VIS
            my_fig.flush();
            my_fig.get_key();
#endif
            clr = false;

            current_time = std::chrono::steady_clock::now();
        }

        else
            return;

        //-------------------------------------------
        if (drawVertex)
        {
#if VIS
            my_fig.addChild(sglBox(n.x, n.y, n.z, n.x + 1, n.y + 1, n.z + 0.2, col[0], col[1], col[2], alpha));
#endif
        }

        if (expand_count % VIS_INTERVAL == 0 || node_heap_p->size() == 0)
        {
#if VIS
            my_fig.flush();
            sleep(0.5);
#endif
        }
    }

    // ---------------------------------------

    bool stopSearch(myNode &n)
    { // Stop conditions
        if (n.isCoordsEqual(goalNode))
        {
// for (auto &g : homotopyGoals)
// {
//     if (n.hasCommonPredecessors(g))
//         return false;
// }
#if VIS
            my_fig.addChild(sglSphere(goalNode.x, goalNode.y, goalNode.z, 0.5, 0, 1, 0));
#endif
            homotopyGoals.push_back(n);
            ++nClasses;
            n.print("Found a path to ");
            current_time = std::chrono::steady_clock::now();
            std::cout << "Found path #" << nClasses << ". Expanded " << expand_count << " vertices. Time since start: " << std::chrono::duration_cast<std::chrono::seconds>(current_time - start_time).count() << "s" << std::endl;
            if (nClasses >= nClassesToFind)
            {
#if VIS
                my_fig.flush();
                my_fig.get_key();
#endif
                return (true);
            }
        }
        return (false);
    }
};

#endif