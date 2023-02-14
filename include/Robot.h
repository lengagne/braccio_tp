#ifndef __ROBOT_H__
#define __ROBOT_H__

// include qui vient de auro8_tp1
#include "Transformation.h"
#include "std_msgs/Float64MultiArray.h"

typedef std_msgs::Float64MultiArray VECTOR;



class Robot{

public:
    Robot();

    // renvoie TRUE si les valeurs de Q restent dans les limites articulaires
    bool check_joint_value( const VECTOR &Q);
    
    // Renvoie la matrice Jacobienne en position
    Eigen::Matrix<double,3,5> ComputeJacobian( const std_msgs::Float64MultiArray& Q);
    
    // Met a jour la pose de chaque repere (TLink) et renvoie le repere de l'effecteur.
    Transformation  ModGeoDirect( const VECTOR & Q);
    
    // Renvoie le vecteur articulaire OUT pour atteindre la pose définie par T
    // Renvoie false si ce n'est pas faisable
    bool  ModGeoInverse( const Transformation& T,
                            VECTOR & OUT);    
    
    // valeur minimale et maximale des articulations
    std::vector<double> qmin,qmax;
    
    // Transformation statiques
    std::vector<Transformation> TStatic;
    
    // Transformation qui vont dépendre des angles.
    std::vector<Transformation> TJoint;
    
    // Transformation des repères
    std::vector<Transformation> TLink;
        
    double PI = 2*asin(1);
};

#endif 
