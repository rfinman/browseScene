#ifndef _convert_poses_hpp_
#define _convert_poses_hpp_

#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fstream>
#include <vector>
#include <sstream>
#include <iomanip>

class ConvertPoses
{
    public:
    ConvertPoses(std::string);
    ConvertPoses(std::string, std::string);
    ~ConvertPoses();

    std::vector<float> readPoseFromFile();
    bool writePose(std::vector<float>);
    void closeFile();
    private:
    bool has_file;
    std::ifstream infile;
    std::ofstream outfile;
    int frame_num;

    /* Set local paths */
    static const std::string povray_path;
    static const std::string povray_include_path;
};

#endif /* _convert_poses_hpp_ */
