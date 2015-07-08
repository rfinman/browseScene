#include "convert_poses.hpp"

const std::string ConvertPoses::povray_path = "/home/rfinman/software/synthetic_data/scratch/workspace/povray.3.7.0.rc3.withdepthmap/unix/povray";
const std::string ConvertPoses::povray_include_path = "/home/rfinman/software/synthetic_data/scratch/workspace/povray.3.7.0.rc3.withdepthmap/include";
 

ConvertPoses::ConvertPoses(std::string in, std::string out):
                                infile(in.c_str()),
                                outfile(out.c_str()),
                                has_file(true),
                                frame_num(0)
{
    //outfile.open("povray_script.sh");
    if (!infile.is_open())
        std::cout<<"Couldn't open file "<<infile<<std::endl;

    if (!outfile.is_open())
        std::cout<<"Couldn't open file "<<outfile<<std::endl;


}

ConvertPoses::ConvertPoses(std::string out):
                    outfile(out.c_str()),
                    has_file(false),
                    frame_num(0)
{
    //outfile.open("povray_script.sh");
    if (!outfile.is_open())
        std::cout<<"Couldn't open file "<<outfile<<std::endl;
}
ConvertPoses::~ConvertPoses()
{

}

std::vector<float> ConvertPoses::readPoseFromFile()
{
    std::vector<float> pose;
    if (!has_file)
    {
        std::cout << "No input file found to read from"<<std::endl;
        return pose;
    }

    std::string line;
    std::string token;
    for (int i = 0; i < 3; i++)
    {
        if (!getline(infile, line))
        {
            pose.clear();
            return pose;

        }
        std::stringstream lineStream(line);
        while (lineStream >> token)
        {
            pose.push_back(atof(token.c_str()));
        }
    }
    // Remove empty line
    getline(infile, line);

    return pose;
}

bool ConvertPoses::writePose(std::vector<float> pose)
{
    if (pose.size() != 12)
    {
        std::cout<<"invalid pose"<<std::endl;
        std::cout<< "Expected 12 values but got "<<pose.size()<<std::endl;
        return false;
    }

    int i = 0;
    outfile << ConvertPoses::povray_path;
    outfile << " +Iliving_room.pov +WT12 +Oscene_";
    outfile << std::setfill('0') << std::setw(5) << frame_num++ <<".png ";
    outfile << " +W640 +H480";
    outfile << " + Declare=val00="<<pose[i++];
    outfile << " + Declare=val01="<<pose[i++];
    outfile << " + Declare=val02="<<pose[i++];
    outfile << " + Declare=val10="<<pose[i++];
    outfile << " + Declare=val11="<<pose[i++];
    outfile << " + Declare=val12="<<pose[i++];
    outfile << " + Declare=val20="<<pose[i++];
    outfile << " + Declare=val21="<<pose[i++];
    outfile << " + Declare=val22="<<pose[i++];
    outfile << " + Declare=val30="<<pose[i++];
    outfile << " + Declare=val31="<<pose[i++];
    outfile << " + Declare=val32="<<pose[i];
    outfile << " +FN16 +wt12 -d +L";
    outfile << ConvertPoses::povray_include_path;
    outfile << " + Declare=use_baking=2"<<std::endl;

    return true;
}

void ConvertPoses::closeFile()
{
    outfile.close();
}






