#include <iostream>
#include <string>
#include <vector>
#include "Scene.h"
#include "Matrix4.h"
#include "Helpers.h"
#include <cmath>

using namespace std;
# define M_PI  3.14159265358979323846
Scene *scene;

struct Line{
    int colorId1, colorId2;
    double x0, x1, y0, y1; 
};


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
Matrix4 cameraTransformation(Camera cam){
    Matrix4 result=getIdentityMatrix();
    double ex=cam.pos.x;
    double ey=cam.pos.y;
    double ez=cam.pos.z;
    result.val[0][0] = cam.u.x;
    result.val[0][1] = cam.u.y;
    result.val[0][2] = cam.u.z;
    result.val[0][3] = -1*(cam.u.x*ex+cam.u.y*ey+cam.u.z*ez);
    result.val[1][0] = cam.v.x;
    result.val[1][1] = cam.v.y;
    result.val[1][2] = cam.v.z;
    result.val[1][3] = -1*(cam.v.x*ex+cam.v.y*ey+cam.v.z*ez);
    result.val[2][0] = cam.w.x;
    result.val[2][1] = cam.w.y;
    result.val[2][2] = cam.w.z;
    result.val[2][3] = -1*(cam.w.x*ex+cam.w.y*ey+cam.w.z*ez);
    return result;

}
// persepective to orth.
Matrix4 p2o(double f, double n){
    Matrix4 result=getIdentityMatrix();
    result.val[0][0]=n;
    result.val[1][1]=n;
    result.val[2][2]=f+n;
    result.val[2][3]=f*n;
    result.val[3][2]=-1;
    result.val[3][3]=0;
    return result;
}
// viewport transformation
// 3x4 matrix is mentioned 
// but extra row does not cause error wrt. slides
Matrix4 viewportT(int nx, int ny){
    Matrix4 result=getIdentityMatrix();
    result.val[0][0]=nx/2;
    result.val[0][3]=(nx-1)/2;
    result.val[1][1]=ny/2;
    result.val[1][3]=(ny-1)/2;
    result.val[2][2]=0.5;
    result.val[2][3]=0.5;
    return result;
    // NOT FINISHED
    // result.val[0][3]=(nx-1)/2

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
    const double cosinus = cos(rotation.angle*(3.14159265f / 180.0f));
    const double sinus = sin(rotation.angle*(3.14159265f / 180.0f));
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
vector<PerMeshModelling> modellingTransformationsPipeline(){
    
    
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
                transformationsAll.insert(transformationsAll.begin(),(tMatrix));
            }
            else if (scene->meshes[i]->transformationTypes[j]=='r'){
                Rotation* trans = scene->rotations[scene->meshes[i]->transformationIds[j]-1];
                Matrix4 tMatrix=rotationMatrix(*trans);
                transformationsAll.insert(transformationsAll.begin(),(tMatrix));
            }
            else if (scene->meshes[i]->transformationTypes[j]=='s'){
                // scalings are not translated
                Scaling* trans=(scene->scalings[scene->meshes[i]->transformationIds[j]-1]);
                Matrix4 tMatrix=scalingMatrix(*trans);
                transformationsAll.insert(transformationsAll.begin(),(tMatrix));

            }
            
        }
        Matrix4 tComposite=compositeAll(transformationsAll);
        PerMeshModelling tP;
        tP.compositeModelling=tComposite;
        tP.mesh_id=i+1;
        perMeshModellings.push_back(tP);
    }
    return perMeshModellings;
}
uint8_t clip(double c){
    if(c > 255) return 255;
    if(c < 0) return 0;
    return c;
}


void drawTriangle(Vec4 vertex0, Vec4 vertex1, Vec4 vertex2,Camera cam){
    double xMin=min(vertex0.x,min(vertex1.x,vertex2.x));
    double yMin=min(vertex0.y,min(vertex1.y,vertex2.y));
    double zMin=min(vertex0.z,min(vertex1.z,vertex2.z));
    double xMax=max(vertex0.x,max(vertex1.x,vertex2.x));
    double yMax=max(vertex0.y,max(vertex1.y,vertex2.y));
    double zMax=max(vertex0.z,max(vertex1.z,vertex2.z));
    Color c0=*(scene->colorsOfVertices[vertex0.colorId-1]);
    Color c1=*(scene->colorsOfVertices[vertex1.colorId-1]);
    Color c2=*(scene->colorsOfVertices[vertex2.colorId-1]);
    for (int y=yMin; y<yMax+1; y++){
        for (int x=xMin; x<xMax+1; x++){
            double alfa=(x*(vertex1.y-vertex2.y)+ y*(vertex2.x-vertex1.x)+vertex1.x*vertex2.y-vertex1.y*vertex2.x)/
                        (vertex0.x*(vertex1.y-vertex2.y)+ vertex0.y*(vertex2.x-vertex1.x)+vertex1.x*vertex2.y-vertex1.y*vertex2.x);
            double beta=(x*(vertex2.y-vertex0.y)+ y*(vertex0.x-vertex2.x)+vertex2.x*vertex0.y-vertex2.y*vertex0.x)/
                        (vertex1.x*(vertex2.y-vertex0.y)+ vertex1.y*(vertex0.x-vertex2.x)+vertex2.x*vertex0.y-vertex2.y*vertex0.x);
            double gamma=(x*(vertex0.y-vertex1.y)+ y*(vertex1.x-vertex0.x)+vertex0.x*vertex1.y-vertex0.y*vertex1.x)/
                        (vertex2.x*(vertex0.y-vertex1.y)+ vertex2.y*(vertex1.x-vertex0.x)+vertex0.x*vertex1.y-vertex0.y*vertex1.x);
            if (alfa>=0 && beta>=0 && gamma>=0){
                int cr=alfa*c0.r+beta*c1.r+gamma*c2.r;
                int cg=alfa*c0.g+beta*c1.g+gamma*c2.g;
                int cb=alfa*c0.b+beta*c1.b+gamma*c2.b;
                if (x>=0 && y>=0 && x<=cam.horRes-1 && y<=cam.verRes-1){
                    scene->image[x][y].r=clip(cr);
                    scene->image[x][y].g=clip(cg);
                    scene->image[x][y].b=clip(cb);
                }
                    
            }
        }

    }

}

bool change_dimension(double& x0, double& y0, double& x1, double& y1, int flag){
    cout <<flag<< " in change dimension" << endl;
    double temp = x0;
    x0 = y0;
    y0 = temp;
    temp = x1;
    x1 = y1;
    y1 = temp;
    return true;
}

bool change_color_id(int& id1, int& id2){
    int temp = id1;
    id1 = id2;
    id2 = temp;
    return true;
}


void draw_line(Line line){
    bool flag=abs(line.y1 - line.y0) > abs(line.x1- line.x0);
    flag && change_dimension(line.x0, line.y0, line.x1, line.y1, flag);
    (line.x0>line.x1) && change_dimension(line.x0, line.x1, line.y0, line.y1, (line.x0>line.x1)) && change_color_id(line.colorId1, line.colorId2); 
    int delta=1;
    if (line.y0>line.y1){
        delta=-1;
    }
    double y = line.y0;
    double d = (-1)*abs(line.y0 - line.y1) + 0.5*(line.x1 - line.x0);
    double inv_diff = 1/(line.x1 - line.x0);
    double  cr = scene->colorsOfVertices[line.colorId1 - 1]->r, 
            cg = scene->colorsOfVertices[line.colorId1 - 1]->g, 
            cb = scene->colorsOfVertices[line.colorId1 - 1]->b;
    double  dcr = (scene->colorsOfVertices[line.colorId2 - 1]->r - cr)*inv_diff, 
            dcg = (scene->colorsOfVertices[line.colorId2 - 1]->g - cg)*inv_diff,
            dcb = (scene->colorsOfVertices[line.colorId2 - 1]->b - cb)*inv_diff;
    for (int i = line.x0; i <= (int)line.x1 && i < scene->image.size(); i++)
    {
        if(y<scene->image[i].size()){
            if (!flag){
                scene->image[i][(int)y].r = clip(cr);
                scene->image[i][(int)y].g = clip(cg);
                scene->image[i][(int)y].b = clip(cb);
            }
            else{
                scene->image[(int)y][i].r = clip(cr);
                scene->image[(int)y][i].g = clip(cg);
                scene->image[(int)y][i].b = clip(cb);
            }
            
        }
        if(d < 0){
            y = y + delta;
            d += (-1)*abs(line.y0 -line.y1) + (line.x1 -line.x0);
        } else {
            d += (-1)*abs(line.y0 - line.y1);
        }
        cr += dcr;
        cg += dcg;
        cb += dcb;
    }
}



bool is_visible(float d, float num, float& t_e, float& t_l ){
    float t = 0;
    if (d > 0){//potentially entering 
        t = num/d;
        if(t > t_l){return false;}
        if(t > t_e){t_e = t;}
    } else if (d < 0) {//potentially leaving
        t = num/d;                
        if(t < t_e){return false;}
        if(t < t_l){t_l = t;}
    } else if (num > 0){//line parallel to edge
        return false;
    }
    return true;

}

Vec4 subtractVec4(Vec4 a, Vec4 b)
{
    Vec4 result;
    result.x = a.x - b.x;
    result.y = a.y - b.y;
    result.z = a.z - b.z;

    return result;
}


Vec4 crossProductVec4(Vec4 a, Vec4 b)
{
    Vec4 result;

    result.x = a.y * b.z - b.y * a.z;
    result.y = b.x * a.z - a.x * b.z;
    result.z = a.x * b.y - b.x * a.y;

    return result;
}

double magnitudeOfVec4(Vec4 v)
{
    return sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

Vec4 normalizeVec4(Vec4 v)
{
    Vec4 result;
    double d;

    d = magnitudeOfVec4(v);
    result.x = v.x / d;
    result.y = v.y / d;
    result.z = v.z / d;

    return result;
}

double dotProductVec4(Vec4 a, Vec4 b)
{
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

bool culling(Vec4 vec1, Vec4 vec2, Vec4 vec3){
	Vec4 n = crossProductVec4(subtractVec4(vec3, vec1), subtractVec4(vec2, vec1));
	Vec4 normalized_n = normalizeVec4(n);
	Vec4 v_viewing = normalizeVec4(vec1);
	return dotProductVec4(v_viewing, normalized_n) > 0;
}


void liang_barsky(Vec4 vec1, Vec4 vec2, Camera* cam){
    // if(vec2.y < vec1.y){
    //     std::swap(vec2, vec1);
    // }
    float t_e = 0, t_l = 1;
    int   x_min = 0, 
            x_max = cam->horRes - 1, 
            y_min = 0, 
            y_max = cam -> verRes - 1;
    bool visible = false; 
    float dx = vec2.x - vec1.x;
    float dy = vec2.y - vec1.y;
    if (is_visible(dx, x_min - vec1.x,t_e, t_l)){ // left
        if (is_visible((-1)*dx, vec1.x - x_max, t_e, t_l)){ // right
            if(is_visible(dy, y_min - vec1.y, t_e, t_l)){ // bottom
                if(is_visible((-1)*dy, vec1.y - y_max, t_e, t_l)){ // top
                    visible = true;
                    if(t_l < 1){
                        vec2.x = vec1.x + dx*t_l;
                        vec2.y = vec1.y + dy*t_l;
                    }
                    if(t_e > 0){
                        vec1.x = vec1.x + dx*t_e;
                        vec1.y = vec1.y + dy*t_e;
                    }        
                    Line res;       
                    res.colorId1 = vec1.colorId, res.colorId2 = vec2.colorId;
                    res.x0 = vec1.x, res.x1 = vec2.x, res.y0 = vec1.y, res.y1 = vec2.y;
                    draw_line(res);
                }
            }
        }
    }
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
            // initialize image with basic values
            scene->initializeImage(scene->cameras[i]);
            // cannot determine how to get gaze x up 
            // gaze w
            // up v
            // u=w x v

            Vec3 gaze=scene->cameras[i]->gaze;
            Vec3 up=scene->cameras[i]->u;
            vector<PerMeshModelling> permeshModelling=modellingTransformationsPipeline();
            Matrix4 cameraTransformationMatrix=cameraTransformation(*(scene->cameras[i]));
            Matrix4 p2oMatrix=getIdentityMatrix();
            Matrix4 orthMatrix=M_orth(  scene->cameras[i]->left, 
                                        scene->cameras[i]->right, 
                                        scene->cameras[i]->bottom, 
                                        scene->cameras[i]->top, 
                                        scene->cameras[i]->near, 
                                        scene->cameras[i]->far);
            Matrix4 viewportMatrix=viewportT(scene->cameras[i]->horRes,scene->cameras[i]->verRes);
            if (scene->cameras[i]->projectionType==1){
                p2oMatrix=p2o(scene->cameras[i]->far,scene->cameras[i]->near);
            }

            for (int j=0;  j<permeshModelling.size(); j++){
                Mesh * msh=scene->meshes[j];
                Matrix4 res=multiplyMatrixWithMatrix(cameraTransformationMatrix, permeshModelling[j].compositeModelling);
                Matrix4 res1=multiplyMatrixWithMatrix(p2oMatrix,res);
                res1=multiplyMatrixWithMatrix(orthMatrix,res1);
                for (int k=0; k<msh->numberOfTriangles; k++){
                    Triangle tFace=msh->triangles[k];
                    int id1=tFace.getFirstVertexId();
                    int id2=tFace.getSecondVertexId();
                    int id3=tFace.getThirdVertexId();
                    Vec3* point1=scene->vertices[id1-1];
                    Vec3* point2=scene->vertices[id2-1];
                    Vec3* point3=scene->vertices[id3-1];
                    Vec4 vec1=Vec4(point1->x,point1->y,point1->z,1,point1->colorId);
                    Vec4 vec2=Vec4(point2->x,point2->y,point2->z,1,point2->colorId);
                    Vec4 vec3=Vec4(point3->x,point3->y,point3->z,1,point3->colorId);
                    vec1=multiplyMatrixWithVec4(res1,vec1);
                    vec2=multiplyMatrixWithVec4(res1,vec2);
                    vec3=multiplyMatrixWithVec4(res1,vec3);

                    if (scene->cameras[i]->projectionType==1){
                        vec1.x/=vec1.t;
                        vec1.y/=vec1.t;
                        vec1.z/=vec1.t;
                        vec1.t=1.0f;

                        vec2.x/=vec2.t;
                        vec2.y/=vec2.t;
                        vec2.z/=vec2.t;
                        vec2.t=1.0f;

                        vec3.x/=vec3.t;
                        vec3.y/=vec3.t;
                        vec3.z/=vec3.t;
                        vec3.t=1.0f;
                    }


                    Vec4 final1=multiplyMatrixWithVec4(viewportMatrix,vec1);
                    final1.x = int(final1.x) + 0.5;
                    final1.y = int(final1.y) + 0.5;
                    Vec4 final2=multiplyMatrixWithVec4(viewportMatrix,vec2);
                    final2.x = int(final2.x) + 0.5;
                    final2.y = int(final2.y) + 0.5;
                    Vec4 final3=multiplyMatrixWithVec4(viewportMatrix,vec3);
                    final3.x = int(final3.x) + 0.5;
                    final3.y = int(final3.y) + 0.5;
                    cout<<final1.x<<" "<<final1.y<<" "<<final1.z<<endl;
                    cout<<final2.x<<" "<<final2.y<<" "<<final2.z<<endl;
                    cout<<final3.x<<" "<<final3.y<<" "<<final3.z<<endl;  
                    if (scene->cullingEnabled && culling(final1, final2,final3)){
                        continue;
                    }  
                    Line line1, line2, line3;
                    if (msh->type == 0){
                            liang_barsky(final1, final2, scene->cameras[i]);
                            liang_barsky(final2, final3, scene->cameras[i]);
                            liang_barsky(final3, final1, scene->cameras[i]); 
                    } else{
                        drawTriangle(final1,final2,final3,*scene->cameras[i]);
                    }
                
                }
            }
            
            

            // do forward rendering pipeline operations
           // scene->forwardRenderingPipeline(scene->cameras[i]);

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
