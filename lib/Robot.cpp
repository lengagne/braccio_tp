
#include "Robot.h"


Robot::Robot()
{
    // A MODIFIER
    uint nb_joint = 1;      // définition du nombre d'articulation
    uint nb_body = 1;       // définition du nombre de corps
    TJoint.resize(nb_joint);   // on définit que le vecteur TJoint (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
    TLink.resize(nb_body);     // on définit que le vecteur TLink  (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)    
    TStatic.resize(nb_joint);  // on définit que le vecteur TStatic (vecteur de transformation) va avoir un élément (qu'on ne connait pas encore)
    
    qmin.resize(nb_joint);  // on définit un vecteur de double avec nb_joint composant (en radian)
    qmax.resize(nb_joint);  // on définit un vecteur de double avec nb_joint composant (en radian)
    
    TStatic[0] = Transformation (0,0,0,0,0,0);                // on définit la premiere transformation, les 6 paramètres sont : Tx, Ty, Tz, Roll, Pitch, Yaw  (angle en degré)
    qmin[0] = 0;
    qmax[0] = PI;
    
}

// renvoi la pose de l'effecteur en fonction des variables articulaires Q.
Transformation Robot::ModGeoDirect( const VECTOR& Q)
{
    double q1 = Q.data[0];
    double q2 = Q.data[1];
    double q3 = Q.data[2];
    double q4 = Q.data[3];
    double q5 = Q.data[4];
    
     Transformation Teff;
    // A COMPLETER
     
     return Teff;
}

// Cette fonction calcule les valeurs articulaires pour que l'effecteur atteigne le point définit par le vecteur pos.
// Si le point n'est pas accessible, la fonction renvoie false
bool  Robot::ModGeoInverse( const Eigen::Matrix<double,3,1> & pos,
                            VECTOR & Q)
{    
    Q.data.resize(6);    
    double X = pos(0);
    double Y = pos(1);
    double Z = pos(2);
    
    // A COMPLETER
    

    
    // ne pas oublier de vérifier que les valeurs sont dans les limites articulaires
    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
    {
        if (Q.data[i] < qmin[i]) Q.data[i] = qmin[i];
        if (Q.data[i] > qmax[i]) Q.data[i] = qmax[i];
    }    
    
    // Renvoi vrai si faisable
    return true;
}

// Renvoie la jacobienne (position en 3D d'un point de l'effecteur) pour un vecteur articulaire
Eigen::Matrix<double,3,5> Robot::ComputeJacobian( const VECTOR& Q)
{
    Eigen::Matrix<double,3,5> out;
//     out(0,0) = 
    // ..
//     out(2,4) = 
    return out;
}



// Calcule le vecteur articulaire suivant permettant de se rapprocher de la cible
// entrees
//      Qin : Vecteur articulaire initial
//      CurrentPosition : Position (vecteur 3D) actuelle de l'effecteur (par modèle ou par mesure)
//      DesiredPosition : Position (vecteur 3D) désirée de l'effecteur
// sorties
//      Qout : Vecteur articulaire permettant de se rapprocher de la cible
//      retourne la distance entre la position actuelle et désirée
double Robot::ComputeControl(   const VECTOR& Qin,
                                const Eigen::Matrix<double,3,1>& CurrentPosition,
                                const Eigen::Matrix<double,3,1>& DesiredPosition,
                                VECTOR& Qout)
{
    // on récupère la jacobienne 
    Eigen::Matrix<double,3,5> Jacobian = ComputeJacobian(Qin);

    
    

    for (int i=0;i<5;i++)   // on ne vérifie pas le gripper
    {
        if (Qout.data[i] < qmin[i]) Qout.data[i] = qmin[i];
        if (Qout.data[i] > qmax[i]) Qout.data[i] = qmax[i];
    }           
    
    return 42;  // A COMPLETER
}
