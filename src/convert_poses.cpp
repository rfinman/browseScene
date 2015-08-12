#include "convert_poses.hpp"

const std::string ConvertPoses::povray_path = "/home/rfinman/software/synthetic_data/scratch/workspace/povray.3.7.0.rc3.withdepthmap/unix/povray";
const std::string ConvertPoses::povray_include_path = "/home/rfinman/software/synthetic_data/scratch/workspace/povray.3.7.0.rc3.withdepthmap/include";
const std::string ConvertPoses::directory = "poses/";
const std::string ConvertPoses::pov_header = "living_room_";

ConvertPoses::ConvertPoses(std::string in, std::string out):
    infile(in.c_str()),
    outfile(out.c_str()),
    has_file(true),
    frame_num(0),
    num_cores(sysconf(_SC_NPROCESSORS_ONLN))
{
//    mkdir(ConvertPoses::directory.c_str(), 0x777);    

    if (!infile.is_open())
        std::cout<<"Couldn't open infile "<<infile<<std::endl;

    if (!outfile.is_open())
        std::cout<<"Couldn't open outfile "<<outfile<<std::endl;
}

ConvertPoses::ConvertPoses(std::string out):
    outfile(out.c_str()),
    has_file(false),
    frame_num(0),
    num_cores(sysconf(_SC_NPROCESSORS_ONLN))
{
//    mkdir(ConvertPoses::directory.c_str(), 0x777);    

    if (!outfile.is_open())
        std::cout<<"Couldn't open file "<<outfile<<std::endl;
}

ConvertPoses::~ConvertPoses(){}

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

    outfile << ConvertPoses::povray_path;
    outfile << " +Iliving_room.pov +WT"<<num_cores<<" +Oscene_";
    outfile << std::setfill('0') << std::setw(5) << frame_num++ <<".png ";
    outfile << " +W640 +H480";
    outfile << " + Declare=val00="<<pose[0];
    outfile << " + Declare=val01="<<pose[4];
    outfile << " + Declare=val02="<<pose[8];
    outfile << " + Declare=val10="<<pose[1];
    outfile << " + Declare=val11="<<pose[5];
    outfile << " + Declare=val12="<<pose[9];
    outfile << " + Declare=val20="<<pose[2];
    outfile << " + Declare=val21="<<pose[6];
    outfile << " + Declare=val22="<<pose[10];
    outfile << " + Declare=val30="<<pose[3];
    outfile << " + Declare=val31="<<pose[7];
    outfile << " + Declare=val32="<<pose[11];
    outfile << " +FN16 +wt"<<num_cores<<" -d +L";
    outfile << ConvertPoses::povray_include_path;
    outfile << " + Declare=use_baking=2"<<std::endl;

    return true;
}

bool ConvertPoses::writePoseAndPOVRAY(std::vector<float> pose)
{
    if (pose.size() != 12)
    {
        std::cout<<"invalid pose"<<std::endl;
        std::cout<< "Expected 12 values but got "<<pose.size()<<std::endl;
        return false;
    }

    std::stringstream index;
    index << std::setfill('0') << std::setw(5) << frame_num;

    std::ofstream pov_file((ConvertPoses::directory + 
                ConvertPoses::pov_header + index.str()).c_str());
    std::ofstream geom_file((ConvertPoses::directory + 
                ConvertPoses::pov_header +"POV_geom_" + index.str()).c_str());

    // Set up the .pov file
    if (!pov_file.is_open())
    {
        std::cout<<"Couldn't open file "<<pov_file<<std::endl;
        return false;
    }

    pov_file << "#version 3.7"<<std::endl;
    pov_file << "#include \"living_room_baking_control.inc\""<<std::endl;
    pov_file << "#include \"living_room_POV_geom_"<<index.str()<<std::endl;
    pov_file << "object{living_room_}"<<std::endl;
    pov_file << "#if (use_baking>2)"<<std::endl;
    pov_file << "  #include \"living_room_lights.inc\""<<std::endl;
    pov_file << "#end"<<std::endl;
    pov_file << "#if (use_baking=1)"<<std::endl;
    pov_file << "  #include \"living_room_baking_camera.inc\""<<std::endl;
    pov_file << "#else"<<std::endl;
    pov_file << "  #include \"living_room_camera.inc\""<<std::endl;
    pov_file << "#end"<<std::endl;
    pov_file << "#include \"living_room_unbaked.inc\""<<std::endl;

    pov_file.close();

    // Set up geometry file
    if (!geom_file.is_open())
        std::cout<<"Couldn't open file "<<geom_file<<std::endl;
    std::ifstream starting_geom_file("/home/rfinman/software/browsescene/data/living_room_POV_geom.inc");
    geom_file << starting_geom_file<<std::endl;
    geom_file << "#declare living_room_="<<std::endl;
    geom_file << "union {"<<std::endl;
    geom_file << "object{living_room_room_molding_  material{room_molding_} hollow}"<<std::endl;
    geom_file << "object{living_room_wall_pic3_frame_  material{wall_pic3_frame_} hollow}"<<std::endl;
    geom_file << "object{living_room_wall_pic2_frame_  material{wall_pic2_frame_} hollow}"<<std::endl;
    geom_file << "object{living_room_door_  material{door_} hollow}"<<std::endl;
    geom_file << "object{living_room_chair2_seat_  material{chair2_seat_} hollow}"<<std::endl;
    geom_file << "object{living_room_chair2_legs_  material{chair2_legs_} hollow}"<<std::endl;
    geom_file << "object{living_room_wall_pic3_  material{wall_pic3_} hollow}"<<std::endl;
    geom_file << "object{living_room_wall_pic2_  material{wall_pic2_} hollow}"<<std::endl;
    geom_file << "object{living_room_spots_light_  material{spots_light_} hollow}"<<std::endl;
    geom_file << "object{living_room_spots_metal_  material{spots_metal_} hollow}"<<std::endl;
    geom_file << "object{living_room_balcony_ceiling_  material{balcony_ceiling_} hollow}"<<std::endl;
    geom_file << "object{living_room_balcony_floor_  material{balcony_floor_} hollow}"<<std::endl;
    geom_file << "object{living_room_balcony_wall_  material{balcony_wall_} hollow}"<<std::endl;
    geom_file << "object{living_room_enclosure_  material{enclosure_} hollow}"<<std::endl;
    geom_file << "object{living_room_room_ceiling_  material{room_ceiling_} hollow}"<<std::endl;
    geom_file << "object{living_room_room_floor_  material{room_floor_} hollow}"<<std::endl;
    geom_file << "object{living_room_room_wall1_  material{room_wall1_} hollow}"<<std::endl;
    geom_file << "object{living_room_room_walls_  material{room_walls_} hollow}"<<std::endl;
    geom_file << "object{living_room_curtains_  material{curtains_} hollow}"<<std::endl;
    geom_file << "object{living_room_wall_pic1_  material{wall_pic1_} hollow}"<<std::endl;
    geom_file << "object{living_room_wall_pic1_frame_  material{wall_pic1_frame_} hollow}"<<std::endl;
    geom_file << "object{living_room_curtain_rail_  material{curtain_rail_} hollow}"<<std::endl;
    geom_file << "object{living_room_foot_lamp1_  material{foot_lamp1_} hollow}"<<std::endl;
    geom_file << "object{living_room_windows_frames_  material{windows_frames_} hollow}"<<std::endl;
    geom_file << "object{living_room_stand_  material{stand_} hollow}"<<std::endl;
    geom_file << "object{living_room_chair1_legs_  material{chair1_legs_} hollow}"<<std::endl;
    geom_file << "object{living_room_chair1_seat_  material{chair1_seat_} hollow}"<<std::endl;
    geom_file << "object{living_room_sofa1_legs_  material{sofa1_legs_} hollow}"<<std::endl;
    geom_file << "object{living_room_sofa1_  material{sofa1_} hollow}"<<std::endl;
    geom_file << "object{living_room_sofa2_legs_  material{sofa2_legs_} hollow}"<<std::endl;
    geom_file << "object{living_room_sofa2_  material{sofa2_} hollow}"<<std::endl;
    geom_file << "object{living_room_table_board_  material{table_board_} hollow}"<<std::endl;
    geom_file << "object{living_room_foot_lamp_shade1_  material{foot_lamp_shade1_} hollow}"<<std::endl;
    geom_file << "object{living_room_palm_pot_  material{palm_pot_} hollow}"<<std::endl;
    geom_file << "object{living_room_palm1_  material{palm1_} hollow}"<<std::endl;
    geom_file << "object{living_room_foot_lamp2_  material{foot_lamp2_} hollow}"<<std::endl;
    geom_file << "object{living_room_foot_lamp_shade2_  material{foot_lamp_shade2_} hollow}"<<std::endl;
    geom_file << "object{living_room_vase1_  material{vase1_} hollow}"<<std::endl;
    geom_file << "object{living_room_cushion_  material{cushion_} hollow}"<<std::endl;
    geom_file << "object{living_room_cushion2_  material{cushion2_} hollow}"<<std::endl;
    geom_file << "object{living_room_cushion3_  material{cushion3_} hollow}"<<std::endl;
    geom_file << "object{living_room_rug_  material{rug_} hollow}"<<std::endl;
    geom_file << "object{living_room_plate_  material{plate_} hollow}"<<std::endl;
    geom_file << "}";


    geom_file.close();



    outfile << ConvertPoses::povray_path;
    outfile << " +I"<<ConvertPoses::pov_header;
    outfile << index.str() << ".pov +WT"<<num_cores<<" +Oscene_";
    outfile << index.str() << ".png +W640 +H480";
    outfile << " + Declare=val00="<<pose[0];
    outfile << " + Declare=val01="<<pose[4];
    outfile << " + Declare=val02="<<pose[8];
    outfile << " + Declare=val10="<<pose[1];
    outfile << " + Declare=val11="<<pose[5];
    outfile << " + Declare=val12="<<pose[9];
    outfile << " + Declare=val20="<<pose[2];
    outfile << " + Declare=val21="<<pose[6];
    outfile << " + Declare=val22="<<pose[10];
    outfile << " + Declare=val30="<<pose[3];
    outfile << " + Declare=val31="<<pose[7];
    outfile << " + Declare=val32="<<pose[11];
    outfile << " +FN16 +wt"<<num_cores<<" -d +L";
    outfile << ConvertPoses::povray_include_path;
    outfile << " + Declare=use_baking=2"<<std::endl;

    return true;
}

void ConvertPoses::closeFile()
{
    outfile.close();
}






