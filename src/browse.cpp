#include <iostream>

#include <iostream>
#include <opencv2/opencv.hpp>
#include <boost/filesystem.hpp>
#include <boost/thread.hpp>

#include<TooN/TooN.h>


#include <pangolin/pangolin.h>
#include <pangolin/display.h>

//#include <OpenNI.h>
#include <cvd/image_io.h>

#include "utils/map_object_label2training_label.h"
#include "utils/se3_file_utils.h"
#include "utils/povray_utils.h"
#include "utils/tiny_obj_loader.h"
#include "joystick/JoystickController.hpp"

using namespace pangolin;

static void PrintInfo(const std::vector<tinyobj::shape_t>& shapes, const std::vector<tinyobj::material_t>& materials)
{
  std::cout << "# of shapes    : " << shapes.size() << std::endl;
  std::cout << "# of materials : " << materials.size() << std::endl;

  for (size_t i = 0; i < shapes.size(); i++) {
    printf("shape[%ld].name = %s\n", i, shapes[i].name.c_str());
//    printf("Size of shape[%ld].indices: %ld\n", i, shapes[i].mesh.indices.size());
//    printf("Size of shape[%ld].material_ids: %ld\n", i, shapes[i].mesh.material_ids.size());
    assert((shapes[i].mesh.indices.size() % 3) == 0);
//    for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++) {
//      printf("  idx[%ld] = %d, %d, %d. mat_id = %d\n", f, shapes[i].mesh.indices[3*f+0], shapes[i].mesh.indices[3*f+1], shapes[i].mesh.indices[3*f+2], shapes[i].mesh.material_ids[f]);
//    }

//    printf("shape[%ld].vertices: %ld\n", i, shapes[i].mesh.positions.size());
//    assert((shapes[i].mesh.positions.size() % 3) == 0);
//    for (size_t v = 0; v < shapes[i].mesh.positions.size() / 3; v++) {
//      printf("  v[%ld] = (%f, %f, %f)\n", v,
//        shapes[i].mesh.positions[3*v+0],
//        shapes[i].mesh.positions[3*v+1],
//        shapes[i].mesh.positions[3*v+2]);
//    }
  }

  for (size_t i = 0; i < materials.size(); i++) {
    printf("material[%ld].name = %s\n", i, materials[i].name.c_str());
    printf("  material.Ka = (%f, %f ,%f)\n", materials[i].ambient[0], materials[i].ambient[1], materials[i].ambient[2]);
    printf("  material.Kd = (%f, %f ,%f)\n", materials[i].diffuse[0], materials[i].diffuse[1], materials[i].diffuse[2]);
    printf("  material.Ks = (%f, %f ,%f)\n", materials[i].specular[0], materials[i].specular[1], materials[i].specular[2]);
    printf("  material.Tr = (%f, %f ,%f)\n", materials[i].transmittance[0], materials[i].transmittance[1], materials[i].transmittance[2]);
    printf("  material.Ke = (%f, %f ,%f)\n", materials[i].emission[0], materials[i].emission[1], materials[i].emission[2]);
    printf("  material.Ns = %f\n", materials[i].shininess);
    printf("  material.Ni = %f\n", materials[i].ior);
    printf("  material.dissolve = %f\n", materials[i].dissolve);
    printf("  material.illum = %d\n", materials[i].illum);
    printf("  material.map_Ka = %s\n", materials[i].ambient_texname.c_str());
    printf("  material.map_Kd = %s\n", materials[i].diffuse_texname.c_str());
    printf("  material.map_Ks = %s\n", materials[i].specular_texname.c_str());
    printf("  material.map_Ns = %s\n", materials[i].normal_texname.c_str());
    std::map<std::string, std::string>::const_iterator it(materials[i].unknown_parameter.begin());
    std::map<std::string, std::string>::const_iterator itEnd(materials[i].unknown_parameter.end());
    for (; it != itEnd; it++) {
      printf("  material.%s = %s\n", it->first.c_str(), it->second.c_str());
    }
    printf("\n");
  }
}

bool TestLoadObj(
  const char* filename,
//  const char* basepath = NULL,
  std::vector<tinyobj::shape_t>& shapes,
  std::vector<tinyobj::material_t>& materials)
{
  std::cout << "Loading " << filename << std::endl;

//  std::vector<tinyobj::shape_t> shapes;
//  std::vector<tinyobj::material_t> materials;
  std::string err = tinyobj::LoadObj(shapes, materials, filename, NULL);

  if (!err.empty()) {
    std::cerr << err << std::endl;
    return false;
  }

  PrintInfo(shapes, materials);

  return true;
}


#define RADPERDEG 0.0174533

void Arrow(GLdouble x1,GLdouble y1,GLdouble z1,GLdouble x2,GLdouble y2,GLdouble z2,GLdouble D)
{
  double x=x2-x1;
  double y=y2-y1;
  double z=z2-z1;
  double L=sqrt(x*x+y*y+z*z);

    GLUquadricObj *quadObj;

    glPushMatrix ();

      glTranslated(x1,y1,z1);

      if((x!=0.)||(y!=0.)) {
        glRotated(atan2(y,x)/RADPERDEG,0.,0.,1.);
        glRotated(atan2(sqrt(x*x+y*y),z)/RADPERDEG,0.,1.,0.);
      } else if (z<0){
        glRotated(180,1.,0.,0.);
      }

      glTranslatef(0,0,L-4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, 2*D, 0.0, 4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, 2*D, 32, 1);
      gluDeleteQuadric(quadObj);

      glTranslatef(0,0,-L+4*D);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluCylinder(quadObj, D, D, L-4*D, 32, 1);
      gluDeleteQuadric(quadObj);

      quadObj = gluNewQuadric ();
      gluQuadricDrawStyle (quadObj, GLU_FILL);
      gluQuadricNormals (quadObj, GLU_SMOOTH);
      gluDisk(quadObj, 0.0, D, 32, 1);
      gluDeleteQuadric(quadObj);

    glPopMatrix ();

}
void drawAxes(GLdouble length)
{
    glPushMatrix();
    glColor3f(1.0,0,0);
    glTranslatef(-length,0,0);
    Arrow(0,0,0, 2*length,0,0, 0.1);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.0,1.0,0);
    glTranslatef(0,-length,0);
    Arrow(0,0,0, 0,2*length,0, 0.1);
    glPopMatrix();

    glPushMatrix();
    glColor3f(0.0,0.0,1.0);
    glTranslatef(0,0,-length);
    Arrow(0,0,0, 0,0,2*length, 0.1);
    glPopMatrix();
}


void change_basis(TooN::SE3<>& T_wc_ref,
//                  TooN::Cholesky<4>& Tchangebasis,
                  TooN::Matrix<4>&T)
{
    TooN::Matrix<4>T4x4 = T.T() * T_wc_ref * T  ;

    TooN::Matrix<3>R_slice = TooN::Data(T4x4(0,0),T4x4(0,1),T4x4(0,2),
                                        T4x4(1,0),T4x4(1,1),T4x4(1,2),
                                        T4x4(2,0),T4x4(2,1),T4x4(2,2));


    TooN::Vector<3>t_slice = TooN::makeVector(T4x4(0,3),T4x4(1,3),T4x4(2,3));

    T_wc_ref = TooN::SE3<>(TooN::SO3<>(R_slice),t_slice);

}


void setOpenglMatrix(OpenGlMatrix& openglSE3Matrix, TooN::SE3<>&T_wcam)
{
    TooN::SE3<>T_cw = T_wcam.inverse();

    TooN::SO3<>Rot = TooN::SO3<>(T_cw.get_rotation()) /** TooN::SO3<>(TooN::makeVector((float)rx,(float)ry,(float)rz))*/;
    TooN::Matrix<3>SO3Mat = Rot.get_matrix();
    TooN::Vector<3>trans = T_cw.get_translation();

    TooN::Matrix<4>SE3Mat = Identity(4);

    SE3Mat.slice(0,0,3,3) = SO3Mat;

    SE3Mat(0,3) = trans[0]/*+(float)tx*/;
    SE3Mat(1,3) = trans[1]/*+(float)ty*/;
    SE3Mat(2,3) = trans[2]/*+(float)tz*/;

    /// Ref: http://www.felixgers.de/teaching/jogl/generalTransfo.html
    /// It should be a transpose - stored in column major
    for(int col = 0; col < 4; col++ )
    {
        for(int row = 0; row < 4; row++)
        {
            openglSE3Matrix.m[col*4+row] = SE3Mat(row,col);
        }
    }
}

double checkDisplacement(TooN::SE3<>&T1, TooN::SE3<>&T2)
{
    TooN::Vector<6>t1_ln = T1.ln();
    TooN::Vector<6>t2_ln = T2.ln();

    return TooN::norm(t1_ln-t2_ln);
}

int main(int argc, char *argv[])
{

    if ( argc < 2)
    {
        std::cerr<<"Usage: ./binary_name obj_file_path " << std::endl;
        std::cerr<<"example: ./tinyobj_browse_camera ../data/room_89_simple.obj" << std::endl;
        exit(1);
    }

    std::string obj_basename(argv[1]);
    std::size_t find_dot = obj_basename.find(".obj");
    std::size_t find_slash = obj_basename.find_last_of('/');

    std::cout<<"find_dot = " << find_dot << std::endl;
    std::cout<<"find_slash = " << find_slash << std::endl;

    obj_basename = obj_basename.substr(find_slash+1,find_dot-find_slash-1);

    std::cout<<"obj_basename = " << obj_basename << std::endl;


    std::string data_dir = "../data/" + obj_basename + "_data";

    /// Simple OBJ reader - main code begins after this..

    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    TestLoadObj(argv[1],shapes,materials);

    std::vector<float*>shape_vertices(shapes.size(),NULL);

    /// Reading the obj mesh
    for(int i = 0; i < shape_vertices.size(); i++)
    {
        int num_vertices = shapes[i].mesh.positions.size()/3;
        int num_faces    = shapes[i].mesh.indices.size() / 3;

        shape_vertices[i] = new float[num_faces*3*3];

        int count=0;

        for (size_t f = 0; f < shapes[i].mesh.indices.size() / 3; f++)
        {
            int v1_idx = shapes[i].mesh.indices[3*f+0];
            int v2_idx = shapes[i].mesh.indices[3*f+1];
            int v3_idx = shapes[i].mesh.indices[3*f+2];

            int max_index = max(max(v1_idx,v2_idx),v3_idx);

            if ( max_index > num_vertices )
            {
                std::cerr<<"max_vertex_index exceeds the number of vertices, something fishy!" << std::endl;
                return 1;
            }

            shape_vertices[i][count+0] = shapes[i].mesh.positions[3*v1_idx+0];
            shape_vertices[i][count+1] = shapes[i].mesh.positions[3*v1_idx+1];
            shape_vertices[i][count+2] = shapes[i].mesh.positions[3*v1_idx+2];

            count+=3;

            shape_vertices[i][count+0] = shapes[i].mesh.positions[3*v2_idx+0];
            shape_vertices[i][count+1] = shapes[i].mesh.positions[3*v2_idx+1];
            shape_vertices[i][count+2] = shapes[i].mesh.positions[3*v2_idx+2];

            count+=3;

            shape_vertices[i][count+0] = shapes[i].mesh.positions[3*v3_idx+0];
            shape_vertices[i][count+1] = shapes[i].mesh.positions[3*v3_idx+1];
            shape_vertices[i][count+2] = shapes[i].mesh.positions[3*v3_idx+2];

            count+=3;

        }
    }


    //#define _joystick_
    #ifdef _joystick_
    /* Start Joystick */
    JoystickController joystick;
    boost::thread* joystickThread = new
    boost::thread(boost::bind(&JoystickController::startJoystick, &joystick));
    #endif

    /// Scale 1 means 640x480 images
    /// Scale 2 means 320x240 images

    int scale              = 1;

    int width              = 640/scale;
    int height             = 480/scale;



    int w_width  = 640;
    int w_height = 480;
    const int UI_WIDTH = 150;

    pangolin::CreateGlutWindowAndBind("GUISandbox",w_width+150,w_height);

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glewInit();

    /// Create a Panel
    pangolin::View& d_panel = pangolin::CreatePanel("ui")
            .SetBounds(1.0, 0.0, 0, pangolin::Attach::Pix(150));

    pangolin::OpenGlRenderState browsing_cam;
    browsing_cam.SetProjectionMatrix(ProjectionMatrixRDF_BottomLeft(640, 480, 420, 420, 320, 320, 0.1, 1000.0));
    browsing_cam.SetModelViewMatrix(ModelViewLookAt(-0.8,1.5,-1.25, 0,1,-3.14, 
    AxisY));

    pangolin::View& display_browsing_cam = pangolin::Display("cam")
      .SetBounds(0.0, 1, Attach::Pix(UI_WIDTH), 1/*0.5*/, -640.0f/480.0f)
      .SetHandler(new Handler3D(browsing_cam));


    TooN::SE3<>T_move;

    std::cout<<"entering the while loop" << std::endl;

    TooN::Matrix<17,3>colours = TooN::Data(0,0,1.0,
                                           0.9137,0.3490,0.1882,
                                           0,0.8549,0,
                                           0.5843,0,0.9412,
                                           0.8706,0.9451,0.0941,
                                           1.0000,0.8078,0.8078,
                                           0,0.8784,0.8980,
                                           0.4157,0.5333,0.8000,
                                           0.4588,0.1137,0.1608,
                                           0.9412,0.1333,0.9216,
                                           0,0.6549,0.6118,
                                           0.9765,0.5451,0,
                                           0.8824,0.8980,0.7608,
                                           1.0000,0,0,
                                           0.8118,0.7176,0.2706,
                                           0.7922,0.5804,0.5804,
                                           0.4902,0.4824,0.4784);   

    std::vector<TooN::SE3<> >poses2render;

    OpenGlMatrix openglSE3Matrix;

    ofstream ofile;

    std::vector<float>red_colours;
    std::vector<float>green_colours;
    std::vector<float>blue_colours;

    for(int i = 0; i < shapes.size(); i++)
    {
        float red   = (float)rand()/RAND_MAX;
        float green = (float)rand()/RAND_MAX;
        float blue  = (float)rand()/RAND_MAX;

        red_colours.push_back( red );
        green_colours.push_back( green );
        blue_colours.push_back( blue );
    }

    std::vector<TooN::SE3<> > gtPoses;

    TooN::SE3<>T_wc;

    TooN::SE3<>T_prev;

    int count = 0;

    char trajfile[200];

    sprintf(trajfile,"%s/%s_trajectory_random_poses_SE3_3x4.txt",
            data_dir.c_str(),
            obj_basename.c_str());

    std::vector<TooN::SE3<> > poses2plot;


    TooN::SE3<>T_wcam;
    TooN::SE3<>T_saved;
    bool entered = false;

    TooN::SE3<>T_s;

    while(!pangolin::ShouldQuit())
    {

        static Var<float>tx("ui.tx",0,0,0.1);
        static Var<float>ty("ui.ty",0,0,0.1);
        static Var<float>tz("ui.tz",0,0,0.1);

        static Var<float>rx("ui.rx",0,0,0.5);
        static Var<float>ry("ui.ry",0,0,0.5);
        static Var<float>rz("ui.rz",0,0,0.5);

        static Var<float>radius("ui.radius",1,1,10);

        static Var<float> end_pt("ui.end_pt",0.1,0,10);
        static Var<float> line_width("ui.line_width",2,0,100);
        static Var<bool> do_inverse("ui.do_inverse",false);
        static Var<bool> start_browsing("ui.start_browsing",false);
        static Var<bool> plot_3d_model("ui.plot_model",false);
        static Var<bool> plot_trajectory("ui.plot_trajectory",false);
        static Var<bool>video_traj_pose("ui.video_traj",true);
        static Var<bool> set_this_openglMatrix("ui.set_this_openglMatrix",false);
        static Var<bool> add_this_pose("ui.add_this_pose",false);
        static Var<bool>show_camera_and_modelviewgl("ui.show_camera_and_modelviewgl", false);

        static Var<bool>add_pose("ui.add_pose",false);

        static Var<int>numposes2plot("ui.numposes2plot",500,0,500);

        static Var<bool> write_poses("ui.write_poses",false);

        TooN::SE3<> T_co(TooN::SO3<>(TooN::makeVector(0,0,0)),
                               TooN::makeVector(-1,1.5,-1));

        {
            if ( start_browsing )
            {
                if ( count%10 == 0 && checkDisplacement(T_wc,T_prev) > 1E-3 )
                {
                    poses2render.push_back(T_wc);
                    add_this_pose = false;

                    T_prev = T_wc;
                }

                TooN::SE3<>T_ow = T_wc.inverse();


                #ifdef _joystick_
                TooN::SE3<>T_update(TooN::SO3<>(TooN::makeVector(
                                        joystick.getPitch()/100,
                                        joystick.getRHoriz()/-100,
                                        joystick.getRoll()/100)),
                                     TooN::makeVector(
                                        joystick.getLHoriz()/100.0*-1,
                                        joystick.getLVert()/100.0,
                                        (joystick.getRTrigger()
                                        -joystick.getLTrigger())/100.0
                                     ));

                #else
                TooN::SE3<>T_update(TooN::SO3<>(TooN::makeVector(
                                        (float)rx, (float)ry, (float)rz)),
                                     TooN::makeVector(
                                        (float)tx, (float)ty, (float)tz)
                                     );
                #endif

                /* There is a weird offset on the camera.  T_co undoes that.
                   The value of T_co was manually found and is not principled*/

                TooN::SE3<> T_prime = T_ow * T_co * T_update * (T_co.inverse());

                TooN::Matrix<4>SE3Mat = TooN::Identity(4);
                SE3Mat.slice(0,0,3,3) = T_prime.get_rotation().get_matrix();
                SE3Mat(0,3) = T_prime.get_translation()[0];
                SE3Mat(1,3) = T_prime.get_translation()[1];
                SE3Mat(2,3) = T_prime.get_translation()[2];

                /// Ref: http://www.felixgers.de/teaching/jogl/generalTransfo.html
                /// It should be a transpose - stored in column major
                for(int col = 0; col < 4; col++ )
                {
                    for(int row = 0; row < 4; row++)
                    {
                        openglSE3Matrix.m[col*4+row] = SE3Mat(row,col);
                    }
                }

                browsing_cam.SetModelViewMatrix(openglSE3Matrix);

                browsing_cam.Apply();

                count++;

            }
            else
            {
                //poses2render.clear();
            }

            if ( set_this_openglMatrix )
            {
                setOpenglMatrix(openglSE3Matrix,T_wcam);

                browsing_cam.SetModelViewMatrix(openglSE3Matrix);

                browsing_cam.Apply();

                entered = true;
            }

            if (!set_this_openglMatrix && entered)
            {
                T_wc = T_saved;

                entered = false;

                setOpenglMatrix(openglSE3Matrix,T_wc);

                browsing_cam.SetModelViewMatrix(openglSE3Matrix);

                browsing_cam.Apply();
            }

            if (!set_this_openglMatrix && !entered)
            {
                T_saved = T_wc;
            }

            float t1, t2,t3;
            TooN::Matrix<3>RMat;

            display_browsing_cam.ActivateScissorAndClear(browsing_cam);

            float modelviewMatrix[16];
            glGetFloatv(GL_MODELVIEW_MATRIX, modelviewMatrix);

            //std::cout<<"Get T_wc"<<std::endl;
            //int a = 0;
            for(int c = 0; c < 3; c++)
            {
                for(int r = 0; r < 3; r++ )
                {
                    RMat(c,r) = modelviewMatrix[r*4+c];
                    //std::cout<<modelviewMatrix[a++]<<", ";
                }
                //std::cout<<modelviewMatrix[a++]<<"  -  "<<(a-1)<<std::endl;
            }
                //std::cout<<std::endl;

            t1 = modelviewMatrix[12];
            t2 = modelviewMatrix[13];
            t3 = modelviewMatrix[14];
            //std::cout <<"trans = "<<t1<<" "<<t2<<" "<<t3<<std::endl;

            TooN::SE3<>currentT_cw = TooN::SE3<>(TooN::SO3<>(RMat),TooN::makeVector(t1,t2,t3));
            //std::cout<<currentT_cw<<std::endl;
            T_wc = currentT_cw.inverse();
            //std::cout<<T_wc<<std::endl;

            glEnable(GL_DEPTH_TEST);
            glClear(GL_COLOR_BUFFER_BIT);
            glColor3f(1.0f,1.0f,1.0f);

            /// Code to plot each of the objects in the mesh
            if( plot_3d_model )
            {
                for(int i = 0 ; i < shapes.size();i++)
                {

                    int training_label = obj_label2training_label(shapes[i].name);

//                    if ( shapes[i].name.find("curtain") != std::string::npos )
//                    {
//                        std::cout<<shapes[i].name << std::endl;
//                        std::cout<<"training_label = " << training_label<<std::endl;
//                    }

                    glColor3f(colours(training_label,0),colours(training_label,1),colours(training_label,2));

                    glEnableClientState(GL_VERTEX_ARRAY);

                    glVertexPointer(3,GL_FLOAT,0,shape_vertices[i]);
                    glDrawArrays(GL_TRIANGLES,0,shapes[i].mesh.indices.size());
                    glDisableClientState(GL_VERTEX_ARRAY);
                }

                T_wcam = TooN::SE3<>(TooN::SO3<>(TooN::makeVector((float)rx,
                                                                  (float)ry,
                                                                  (float)rz)),
                                                 TooN::makeVector((float)tx,
                                                                  (float)ty,
                                                                  (float)tz));

               // std::cout<<"T_wcam = " << T_wcam << std::endl;
/*
                T_wcam = TooN::SE3<>(TooN::SO3<>(TooN::makeVector((float)joystick.getRVert()/100,
                                                                  (float)joystick.getRHoriz()/100,
                                                                  (float)rz)),
                                                 TooN::makeVector((float)tx,
                                                                  (float)ty,
                                                                  (float)tz));
*/
                povray_utils::DrawCamera(T_wcam, (float)end_pt, (float)line_width,false);

//                if ( !entered )
//                {
//                    T_s = T_wcam;
//                    povray_utils::DrawCamera(T_wcam, (float)end_pt, (float)line_width,false);

//                }

//                if (entered)
//                {
//                    povray_utils::DrawCamera(T_s,(float)end_pt, (float)line_width,false);
//                }

                if ( show_camera_and_modelviewgl )
                {
                    std::cout<<"draw camera = " << std::endl;
                    std::cout<< T_wcam << std::endl;

                    TooN::Matrix<3>_rMat;

                    for(int c = 0; c < 3; c++)
                    {
                        for(int r = 0; r < 3; r++ )
                        {
                            _rMat(c,r) = modelviewMatrix[r*4+c];
                        }
                    }

                    t1 = modelviewMatrix[12];
                    t2 = modelviewMatrix[13];
                    t3 = modelviewMatrix[14];

                    TooN::SE3<>thisT_wc = TooN::SE3<>(TooN::SO3<>(_rMat),TooN::makeVector(t1,t2,t3));
                    thisT_wc = thisT_wc.inverse();

                    std::cout<<"T_wc = " << std::endl;
                    std::cout<<thisT_wc<<std::endl;

                    std::cout<<"oldT_wc = " << std::endl;
                    std::cout<<T_s << std::endl;

                    show_camera_and_modelviewgl = false;
                }

            }

            if ( plot_trajectory )
            {
                for(int i = 0; i < poses2plot.size(); i+=50 )
                {
                    TooN::SE3<>T_wc = poses2plot.at(i);
                    povray_utils::DrawCamera(T_wc,(float)end_pt,(float)line_width,false);
                }

            }

            if ( write_poses )
            {
                char trajectory_fileName[200];

                sprintf(trajectory_fileName,"%s/%s_trajectory_random_poses_SE3_3x4.txt",
                        //data_dir.c_str(),
                        ".",
                        obj_basename.c_str());

                std::cout<<"trajectory_fileName = " << trajectory_fileName << std::endl;

                ofile.open(trajectory_fileName);

                for(int i = 0 ;i < poses2render.size(); i++)
                    ofile << poses2render.at(i) << std::endl;

                ofile.close();

                write_poses = false;

                return 1;
            }

            static Var<int> max_grid_unit("ui.grid_max",10,0,100);
            float grid_max = (int)max_grid_unit;

            const float sizeL = 3.f;
            const float grid = 2.f;

            glPointSize(sizeL);
            glBegin(GL_LINES);
            glColor3f(.25,.25,.25);
            for(float i=-grid_max; i<=grid_max; i+=grid)
            {
                glVertex3f(-grid_max, i, 0.f);
                glVertex3f(grid_max, i, 0.f);

                glVertex3f(i, -grid_max, 0.f);
                glVertex3f(i, grid_max, 0.f);
            }
            glEnd();


        }

        d_panel.Render();
        pangolin::FinishGlutFrame();

    }

}
