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

//     rotation(0,0) =
//     rotation(0,1) =
//     rotation(0,2) =

//     rotation(1,0) =
//     rotation(1,1) =
//     rotation(1,2) =

//     rotation(2,0) =
//     rotation(2,1) =
//     rotation(2,2) =
    
    return rotation;
}

// définit la matrice de rotation à partir des informations des quaternions
Eigen::Matrix<double,3,3> setQuaternion( const double & a, const double & b, const double & c, const double &d)
{
    Eigen::Matrix<double,3,3> rotation;

//     rotation(0,0) = 
//     rotation(0,1) = 
//     rotation(0,2) = 
//     rotation(1,0) = 
//     rotation(1,1) = 
//     rotation(1,2) = 
//     rotation(2,0) = 
//     rotation(2,1) = 
//     rotation(2,2) = 
        
    return rotation;
}

// rétourne les angles Roll, Pitch et Yaw à partir d'une matrice de rotation
Eigen::Matrix<double,3,1> getRPY(const Eigen::Matrix<double,3,3> mat)
{
    Eigen::Matrix<double,3,1> out;   
//     out(0) = 
//     out(1) = 
//     out(2) = 
    
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
//     out.rotation = 
//     out.position = 
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
//     out.rotation = 
//     out.position = 
    return out;
}
