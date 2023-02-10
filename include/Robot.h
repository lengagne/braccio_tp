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
    
    // renvoie la distance la plus courte entre tous les corps du robot et l'obstacle (son repère)
    double get_minimal_distance( const Transformation& T);
    
    // Met a jour la pose de chaque repere (TLink) et renvoie le repere de l'effecteur.
    Transformation  ModGeoDirect( const VECTOR & Q);
    
    // Renvoie le vecteur articulaire OUT pour atteindre la pose définie par T
    // Renvoie false si ce n'est pas faisable
    bool  ModGeoInverse( const Transformation& T,
                            VECTOR & OUT);    
    
    // Si on souhaite mettre a jour d'autre pose (liées au robot par exemple)
    void update_additional_pose();
    
    // valeur minimale et maximale des articulations
    std::vector<double> qmin,qmax;
    
    // Transformation statiques
    std::vector<Transformation> TStatic;
    
    // Transformation qui vont dépendre des angles.
    std::vector<Transformation> TJoint;
    
    // Transformation des repères
    std::vector<Transformation> TLink;
    
    // Transformation de repère additionnels (utiles pour le calcul de distance par exemple)
    std::vector<Transformation> TAdd;
    
    double PI = 2*asin(1);
};

#endif 
