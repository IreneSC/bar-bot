#include <eigen3/Eigen/Geometry>
using namespace Eigen;
Affine3d rotation_z(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(0, 0, 1)));
}
Affine3d rotation_x(double theta){
    return Affine3d(AngleAxisd(theta, Vector3d(1, 0, 0)));
}
Affine3d translation(double x, double y, double z){
    return Affine3d(Translation3d(Vector3d(x,y,z)));
}

Affine3d G1(double d, double theta){
    return translation(0,0,d)*rotation_z(theta);
}
Affine3d G2(double theta){
    return rotation_x(-M_PI/2.0)*rotation_z(theta);
}
Affine3d G3(double d, double theta){
    return translation(d,0,0)*rotation_z(theta);
}
Affine3d G4(double d, double theta){
    return translation(d,0,0)*rotation_z(theta);
}
Affine3d G5(double d){
    return rotation_x(M_PI/2.0)*translation(0,0,-d);
}
Affine3d G6(double theta){
    return rotation_z(theta);
}

Affine3d GST(double d1, double theta1, double theta2, double d3, double theta3,
            double d4, double theta4, double d5, double theta6){
    return G1(d1, theta1)*G2(theta2)*G3(d3, theta3)*G4(d4,theta4)*G5(d5)*G6(theta6);
}
