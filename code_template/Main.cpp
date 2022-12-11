#include <iostream>
#include <string>
#include <vector>
#include "Scene.h"
#include "Matrix4.h"
#include "Helpers.h"

using namespace std;

Scene *scene;

Matrix4 compositeAll(vector<Matrix4> tranformsAll){
    Matrix4 result=getIdentityMatrix();
    for (int i=0; i<tranformsAll.size(); i++){
        result=multiplyMatrixWithMatrix(result,tranformsAll[i]);
    }
    return result;
}

// creating orthonormal tranformation matrix
Matrix4 M_orth(double left, double right, double bottom, double top, double near, double far){
    Matrix4 result=getIdentityMatrix();
    result.val[0][0]=2/(right-left);
    result.val[1][1]=2/(top-bottom);
    result.val[2][2]=-2/(far-near);
    result.val[0][3]=-(right+left)/(right-left);
    result.val[1][3]=-(top+bottom)/(top-bottom);
    result.val[2][3]=-(far+near)/(far-near);
    return result;
}




int main(int argc, char *argv[])
{
    if (argc != 2)
    {
        cout << "Please run the rasterizer as:" << endl
             << "\t./rasterizer <input_file_name>" << endl;
        return 1;
    }
    else
    {
        const char *xmlPath = argv[1];

        scene = new Scene(xmlPath);

        for (int i = 0; i < scene->cameras.size(); i++)
        {
            // cannot determine how to get gaze x up 
            // gaze w
            // up v
            // u=w x v

            Vec3 gaze=scene->cameras[i]->gaze;
            Vec3 up=scene->cameras[i]->u;
            if (scene->cameras[i]->projectionType==1){
                // perspective will be applied
            }
            cout<<scene->cameras[i]->u<<endl;
            cout<<scene->cameras[i]->v<<endl;
            cout<<scene->cameras[i]->w<<endl;


            // initialize image with basic values
            scene->initializeImage(scene->cameras[i]);

            // do forward rendering pipeline operations
            scene->forwardRenderingPipeline(scene->cameras[i]);

            // generate PPM file
            scene->writeImageToPPMFile(scene->cameras[i]);

            // Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
            // Notice that os_type is not given as 1 (Ubuntu) or 2 (Windows), below call doesn't do conversion.
            // Change os_type to 1 or 2, after being sure that you have ImageMagick installed.
            scene->convertPPMToPNG(scene->cameras[i]->outputFileName, 99);
        }

        return 0;
    }
}