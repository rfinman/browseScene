#include <GL/glew.h>

#include <GL/freeglut.h>
#include <GL/glut.h>

//#include <fstream>

#include<TooN/TooN.h>
#include<TooN/se3.h>

#include <cvd/gl_helpers.h>

//#include <vector_types.h>
#include <cvd/image_io.h>



using namespace TooN;
using namespace CVD;

using namespace std;

namespace povray_utils{


/*
    TooN::SE3<> computeTpov_cam(std::string& filebasename, int ref_img_no, int which_blur_sample)
    {
        char text_file_name[360];

        sprintf(text_file_name,"%s/scene_%02d_%04d.txt",filebasename.c_str(),
                which_blur_sample,ref_img_no);

        ifstream cam_pars_file(text_file_name);

        char readlinedata[300];

        float4 direction;
        float4 upvector;
        TooN::Vector<3>posvector;


        while(1)
        {
            cam_pars_file.getline(readlinedata,300);

            if ( cam_pars_file.eof())
                break;

            istringstream iss;

            if ( strstr(readlinedata,"cam_dir")!= NULL)
            {
                std::string cam_dir_str(readlinedata);

                cam_dir_str = cam_dir_str.substr(cam_dir_str.find("= [")+3);
                cam_dir_str = cam_dir_str.substr(0,cam_dir_str.find("]"));

                iss.str(cam_dir_str);
                iss >> direction.x ;
                iss.ignore(1,',');
                iss >> direction.y ;
                iss.ignore(1,',') ;
                iss >> direction.z;
                iss.ignore(1,',');
                //cout << direction.x<< ", "<< direction.y << ", "<< direction.z << endl;
                direction.w = 0.0f;

            }

            if ( strstr(readlinedata,"cam_up")!= NULL)
            {

                string cam_up_str(readlinedata);

                cam_up_str = cam_up_str.substr(cam_up_str.find("= [")+3);
                cam_up_str = cam_up_str.substr(0,cam_up_str.find("]"));


                iss.str(cam_up_str);
                iss >> upvector.x ;
                iss.ignore(1,',');
                iss >> upvector.y ;
                iss.ignore(1,',');
                iss >> upvector.z ;
                iss.ignore(1,',');


                upvector.w = 0.0f;

            }

            if ( strstr(readlinedata,"cam_pos")!= NULL)
            {
                string cam_pos_str(readlinedata);

                cam_pos_str = cam_pos_str.substr(cam_pos_str.find("= [")+3);
                cam_pos_str = cam_pos_str.substr(0,cam_pos_str.find("]"));

                iss.str(cam_pos_str);
                iss >> posvector[0] ;
                iss.ignore(1,',');
                iss >> posvector[1] ;
                iss.ignore(1,',');
                iss >> posvector[2] ;
                iss.ignore(1,',');

            }

        }

        /// z = dir / norm(dir)
        Vector<3> z;
        z[0] = direction.x;
        z[1] = direction.y;
        z[2] = direction.z;
        normalize(z);

        /// x = cross(cam_up, z)
        Vector<3> x = Zeros(3);
        x[0] =  upvector.y * z[2] - upvector.z * z[1];
        x[1] =  upvector.z * z[0] - upvector.x * z[2];
        x[2] =  upvector.x * z[1] - upvector.y * z[0];

        normalize(x);

        /// y = cross(z,x)
        Vector<3> y = Zeros(3);
        y[0] =  z[1] * x[2] - z[2] * x[1];
        y[1] =  z[2] * x[0] - z[0] * x[2];
        y[2] =  z[0] * x[1] - z[1] * x[0];

        Matrix<3,3> R = Zeros(3,3);
        R[0][0] = x[0];
        R[1][0] = x[1];
        R[2][0] = x[2];

        R[0][1] = y[0];
        R[1][1] = y[1];
        R[2][1] = y[2];

        R[0][2] = z[0];
        R[1][2] = z[1];
        R[2][2] = z[2];


        return TooN::SE3<>(R, posvector);
    }



    inline void convertNormaltoRGBImage(float4* normals,
            int width,
            int height,
            std::string& fileName)
    {
        CVD::Image<CVD::Rgb<CVD::byte> >rgbImage = CVD::ImageRef(width,height);

        for(int yy = 0; yy < height; yy++ )
        {
            for(int xx = 0; xx < width ; xx++ )
            {
                float4 normal =  normals[yy*width+xx];

                char red   = (normal.x+1.0f)*127.f;
                char green = (normal.y+1.0f)*127.f;
                char blue  = (normal.z+1.0f)*127.f;

                CVD::Rgb<CVD::byte>pixel_colour = CVD::Rgb<CVD::byte>(red,green,blue);

                rgbImage[CVD::ImageRef(xx,yy)] = pixel_colour;
            }
        }

        CVD::img_save(rgbImage,fileName);
    }



    inline void generate_POVRay_command(TooN::SE3<>& T_wc, std::string OutpngFile)
    {

        TooN::Matrix<> RMat = T_wc.get_rotation().get_matrix();
        TooN::Vector<> TMat = T_wc.get_translation();

        TooN::Matrix<>RMatTranspose = RMat.T();

        std::cout << "/scratch/workspace/povray.3.7.0.rc3.withdepthmap/unix/povray +IScene_New_LR2_POV_scene.pov +O"<<OutpngFile.c_str()<<" +W640 +H480 +wt12 ";
        std::cout << "+ Declare=val00="<<RMatTranspose(0,0)<< " + Declare=val01="<<RMatTranspose(0,1) << " + Declare=val02="<< RMatTranspose(0,2);
        std::cout << "+ Declare=val10="<<RMatTranspose(1,0)<< " + Declare=val11="<<RMatTranspose(1,1) << " + Declare=val12="<< RMatTranspose(1,2);
        std::cout << "+ Declare=val20="<<RMatTranspose(2,0)<< " + Declare=val21="<<RMatTranspose(2,1) << " + Declare=val22="<< RMatTranspose(2,2);
        std::cout << "+ Declare=val30="<<TMat[0]         <<"  + Declare=val31="<<TMat[1]          << " + Declare=val32="<< TMat[2];
        std::cout << " -d +L/scratch/workspace/povray.3.7.0.rc3.withdepthmap/include/" << std::endl;
    }
*/
    void DrawCamera(TooN::SE3<> world_from_cam, float end_pt, float line_width , bool do_inverse)
    {
        glPushMatrix();

        Vector<6> rot_trans = world_from_cam.ln();

        static int i = 0;

        world_from_cam = TooN::SE3<>(rot_trans);

        //        if ( do_inverse )
        //        {
        //            glMultMatrix(world_from_cam.inverse());
        //            if ( i == 0 )
        //            {
        //                std::cout << " world_from_cam inverse = " << world_from_cam << std::endl;
        //                i++;
        //            }

        //        }
        //        else
        {
            glMultMatrix(world_from_cam);
        }


        if ( do_inverse)
        {
            glColor3f(1.0,0.0,1.0);
        }
        else
        {
            glColor3f(0.0,1.0,1.0);
        }

        glShadeModel(GL_SMOOTH);
        glTranslatef(0,0,0);
        /// Ref: http://www.alpcentauri.info/glutsolidsphere.htm

        glutSolidSphere(0.02f, 10.0f, 2.0f);

        glLineWidth(line_width);

        glColor3f(1.0,0.0,0.0);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(end_pt, 0.0f, 0.0f);
        glEnd();

        glColor3f(0.0,1.0,0.0);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, end_pt, 0.0f);
        glEnd();

        glColor3f(0.0,0.0,1.0);
        glBegin(GL_LINES);
        glVertex3f(0.0f, 0.0f, 0.0f);
        glVertex3f(0.0f, 0.0f, end_pt);
        glEnd();

        glPopMatrix();

    }

};
