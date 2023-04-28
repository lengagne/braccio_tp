#include "Transformation.h"

// Renvoie la matrice de transformation pour une rotation autour de X
Transformation RotX(double q)
{
    double s = sin(q);
    double c = cos(q);
    Transformation out;
//     out.rotation(0,0) = 
//     out.rotation(0,1) = 
//     out.rotation(0,2) = 
//     out.rotation(1,0) = 
//     out.rotation(1,1) = 
//     out.rotation(1,2) = 
//     out.rotation(2,0) = 
//     out.rotation(2,1) = 
//     out.rotation(2,2) = 
    
    out.rotation(0,0) = 1;
    out.rotation(0,1) = 0;
    out.rotation(0,2) = 0;
    out.rotation(1,0) = 0;
    out.rotation(1,1) = c;
    out.rotation(1,2) = -s;
    out.rotation(2,0) = 0;
    out.rotation(2,1) = s;
    out.rotation(2,2) = c;   
    
    return out;
}

// Renvoie la matrice de transformation pour une rotation autour de Y
Transformation RotY(double q)
{
    double s = sin(q);
    double c = cos(q);    
    Transformation out;
//     out.rotation(0,0) = 
//     out.rotation(0,1) = 
//     out.rotation(0,2) = 
//     out.rotation(1,0) = 
//     out.rotation(1,1) = 
//     out.rotation(1,2) = 
//     out.rotation(2,0) = 
//     out.rotation(2,1) = 
//     out.rotation(2,2) =    
    
    out.rotation(0,0) = c;
    out.rotation(0,1) = 0;
    out.rotation(0,2) = s;
    out.rotation(1,0) = 0;
    out.rotation(1,1) = 1;
    out.rotation(1,2) = 0;
    out.rotation(2,0) = -s;
    out.rotation(2,1) = 0;
    out.rotation(2,2) = c;      

    return out;   
}

// Renvoie la matrice de transformation pour une rotation autour de Z
Transformation RotZ(double q)
{
    double s = sin(q);
    double c = cos(q);    
    Transformation out;
//     out.rotation(0,0) = 
//     out.rotation(0,1) = 
//     out.rotation(0,2) = 
//     out.rotation(1,0) = 
//     out.rotation(1,1) = 
//     out.rotation(1,2) = 
//     out.rotation(2,0) = 
//     out.rotation(2,1) = 
//     out.rotation(2,2) = 
    
    out.rotation(0,0) = c;
    out.rotation(0,1) = -s;
    out.rotation(0,2) = 0;
    out.rotation(1,0) = s;
    out.rotation(1,1) = c;
    out.rotation(1,2) = 0;
    out.rotation(2,0) = 0;
    out.rotation(2,1) = 0;
    out.rotation(2,2) = 1;    

    return out;    
}


// définit la matrice de rotation à partir des informations de Roll, Pitch et Yaw
Eigen::Matrix<double,3,3> setRPY( const double & roll, const double & pitch, const double & yaw)
{
    Eigen::Matrix<double,3,3> rotation;

    const double cr = cos(roll); // ROLL
    const double sr = sin(roll);
    const double cp = cos(pitch); // PITCH
    const double sp = sin(pitch);
    const double cy = cos(yaw); // YAW
    const double sy = sin(yaw);

    rotation(0,0) = cy*cp;
    rotation(0,1) = cy*sp*sr-sy*cr;
    rotation(0,2) = cy*sp*cr+sy*sr;

    rotation(1,0) = sy*cp;
    rotation(1,1) = sy*sp*sr+cy*cr;
    rotation(1,2) = sy*sp*cr-cy*sr;

    rotation(2,0) = -sp;
    rotation(2,1) = cp*sr;
    rotation(2,2) = cp*cr;
    
    return rotation;
}

// définit la matrice de rotation à partir des informations des quaternions
Eigen::Matrix<double,3,3> setQuaternion( const double & a, const double & b, const double & c, const double &d)
{
    Eigen::Matrix<double,3,3> rotation;

    rotation(0,0) = a*a +b*b -c*c - d*d;
    rotation(0,1) = 2*b*c - 2*a*d;
    rotation(0,2) = 2*a*c + 2*b*d;
    rotation(1,0) = 2*a*d + 2*b*c;
    rotation(1,1) = a*a - b*b + c*c - d*d;
    rotation(1,2) = 2*c*d - 2*a*b;
    rotation(2,0) = 2*b*d - 2*a*c;
    rotation(2,1) = 2*a*b + 2*c*d;
    rotation(2,2) = a*a - b*b - c*c + d*d;   
        
    return rotation;
}

// rétourne les angles Roll, Pitch et Yaw à partir d'une matrice de rotation
Eigen::Matrix<double,3,1> getRPY(const Eigen::Matrix<double,3,3> mat)
{
    Eigen::Matrix<double,3,1> out;   
    out(1) = atan2(-mat(2,0),sqrt( pow( mat(0,0),2) + pow( mat(1,0),2)));
    double COS = cos(out(1));
    out(0) = atan2( mat(2,1)/COS , mat(2,2)/COS);
    out(2) = atan2( mat(1,0)/COS , mat(0,0)/COS);
    
    return out;
}

// retourne l'inverse d'une transformation
Transformation Transformation::inverse()
{
    // entree
    //      rotation : matrice 3x3
    //      position : vector 3x1
    // sortie
    //      out.rotation : matrice 3x3
    //      out.position : vector 3x1 
    Transformation out;
    out.rotation = rotation.transpose();
    out.position = - rotation.transpose() * position;
    return out;
}

// retourne le produit de deux transformations
Transformation operator* (const Transformation& A, const Transformation &B)
{
    // entree
    //      A.rotation : matrice 3x3 de la première matrice de transformation
    //      A.position : vector 3x1  de la première matrice de transformation
    //      B.rotation : matrice 3x3 de la deuxième matrice de transformation
    //      B.position : vector 3x1  de la deuxième matrice de transformation    
    // sortie
    //      out.rotation : matrice 3x3
    //      out.position : vector 3x1     
    
    Transformation out;
    out.rotation = A.rotation * B.rotation; 
    out.position = A.position + A.rotation* B.position;     
    return out;
}
