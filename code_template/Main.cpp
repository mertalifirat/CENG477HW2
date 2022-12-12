#include <iostream>
#include <string>
#include <vector>
#include "Scene.h"
#include "Matrix4.h"
#include "Helpers.h"
#include <cmath>

using namespace std;

Scene *scene;
//
// helpers on matrices
//
Matrix4 compositeAll(vector<Matrix4>& tranformsAll){
    Matrix4 result=getIdentityMatrix();
    for (int i=0; i<tranformsAll.size(); i++){
        Matrix4 x = tranformsAll[i];
        result=multiplyMatrixWithMatrix(result,x);
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

Matrix4 translateBack(Matrix4 matrix){
    matrix.val[0][3]*=-1;
    matrix.val[1][3]*=-1;
    matrix.val[2][3]*=-1;
    return matrix;
}


struct PerMeshModelling{
    int mesh_id;
    Matrix4 compositeModelling;
};

// scaling matrix creation
Matrix4 scalingMatrix(Scaling scaling){
    Matrix4 result=getIdentityMatrix();
    result.val[0][0]=scaling.sx;
    result.val[1][1]=scaling.sy;
    result.val[2][2]=scaling.sz;
    return result;
}

// rotation matrix creation
Matrix4 rotationMatrix(Rotation rotation){
    Matrix4 result=getIdentityMatrix();
    const double cosinus = cos(rotation.angle*(3.141/180));
    const double sinus = sin(rotation.angle*(3.141/180));
    result.val[0][0] = cosinus + (rotation.ux)*(rotation.ux)*(1 - cosinus) ;
    result.val[0][1] = (rotation.ux)*(rotation.uy)*(1 - cosinus) - ((rotation.uz)*sinus);
    result.val[0][2] = (rotation.ux)*(rotation.uz)*(1 - cosinus) + (rotation.uy*sinus);
    result.val[1][0] = (rotation.ux)*(rotation.uy)*(1 - cosinus) + ((rotation.uz)*sinus);
    result.val[1][1] = cosinus + (rotation.uy)*(rotation.uy)*(1 - cosinus) ;
    result.val[1][2] = (rotation.uy)*(rotation.uz)*(1 - cosinus) - (rotation.ux*sinus);
    result.val[2][0] = (rotation.ux)*(rotation.uz)*(1 - cosinus) - (rotation.uy*sinus);
    result.val[2][1] = (rotation.uy)*(rotation.uz)*(1 - cosinus) + (rotation.ux*sinus);
    result.val[2][2] = cosinus + (rotation.uz)*(rotation.uz)*(1 - cosinus) ;
    return result;
}

// translation matrix creation
Matrix4 translationMatrix(Translation translation){
    Matrix4 result=getIdentityMatrix();
    result.val[0][3]=translation.tx;
    result.val[1][3]=translation.ty;
    result.val[2][3]=translation.tz;
    return result;
}

// eventually we will get the composite modelli.ng transformations
Matrix4 modellingTransformationsPipeline(){
    
    
    vector<PerMeshModelling> perMeshModellings; 
    // per each mesh
    for (int i=0; i<scene->meshes.size(); i++){
        vector<Matrix4> transformationsAll;
        // each mesh's transformations
        for (int j=0; j<scene->meshes[i]->numberOfTransformations; j++){
            if (scene->meshes[i]->transformationTypes[j]=='t'){
                // from all translations take the one wrt. transformation id of this mesh's tranfsormation
                // id so minus 1 
                Translation* trans=(scene->translations[scene->meshes[i]->transformationIds[j]-1]);
                // tranlation function creation
                Matrix4 tMatrix=translationMatrix(*trans);
                // only translation
                transformationsAll.push_back(tMatrix);
                cout<<tMatrix<<endl;
            }
            else if (scene->meshes[i]->transformationTypes[j]=='r'){
                Rotation* trans = scene->rotations[scene->meshes[i]->transformationIds[j]-1];
                Matrix4 tMatrix=rotationMatrix(*trans);
                transformationsAll.push_back(tMatrix);
                cout<<tMatrix<<endl;
            }
            else if (scene->meshes[i]->transformationTypes[j]=='s'){
                // scalings are not translated
                Scaling* trans=(scene->scalings[scene->meshes[i]->transformationIds[j]-1]);
                Matrix4 tMatrix=scalingMatrix(*trans);
                transformationsAll.push_back(tMatrix);
                cout<<tMatrix<<endl;

            }
            
        }
        Matrix4 tComposite=compositeAll(transformationsAll);
        PerMeshModelling tP;
        tP.compositeModelling=tComposite;
        tP.mesh_id=i+1;
        perMeshModellings.push_back(tP);
        cout << tComposite << endl;

    }
    for (int i=0; i<perMeshModellings.size(); i++){
        cout<<perMeshModellings[i].compositeModelling<<endl;
    }

    return Matrix4();
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
            modellingTransformationsPipeline();
            if (scene->cameras[i]->projectionType==1){
                // perspective will be applied
            }
            // cout<<scene->cameras[i]->u<<endl;
            // cout<<scene->cameras[i]->v<<endl;
            // cout<<scene->cameras[i]->w<<endl;


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