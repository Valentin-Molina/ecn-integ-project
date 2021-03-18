/* Noé Masson - François Lalubin
 *
 * CENTRALE NANTES - Option Robotique 2020-2021
 *
 * Ce script a pour objectif de creer un fichier yaml contenant les parametres géométriques d'un robot RR
 * déterminés apres indentification geometrique / dynamique
 *
 * La saisie des parametres se fait manuellement
 *
 * Il faut preciser la destination o`u creer le fichier yaml `a la ligne de declaration du logger
 *
 *
 * la librairie log2plot peut etre obtenue `a l'adresse suivante :
 * https://github.com/oKermorgant/log2plot.git
 */



#include <iostream>
#include <log2plot/logger.h>

using namespace std;

int main()
{
    double l1,l2,Iz1,Ia1,m1,Iz2,Ia2,m2,m_effecteur, c1, c2; //variables identifiees

    //Saisie a la main des valeurs obtenues en fin d'identification

    std::cout<<"Rentrez la valeur de la variable affichée"<<std::endl;

    std::cout<<"l1 =";
    std::cin>>l1;

    std::cout<<"l2 =";
    std::cin>>l2;

    std::cout<<"Iz1 =";
    std::cin>>Iz1;

    std::cout<<"Ia1 =";
    std::cin>>Ia1;

    std::cout<<"m1 =";
    std::cin>>m1;

    std::cout<<"Iz2 =";
    std::cin>>Iz2;

    std::cout<<"Ia2 =";
    std::cin>>Ia2;

    std::cout<<"m2 =";
    std::cin>>m2;

    std::cout<<"m_effecteur =";
    std::cin>>m_effecteur;

    std::cout<<"c1 =";
    std::cin>>c1;

    std::cout<<"c2 =";
    std::cin>>c2;

    Iz1=Iz1+Ia1;
    Iz2=Iz2+Ia2;


   //Creation du fichier yaml

    log2plot::Logger logger("/home/ecn/");

    logger.save(l1,"l1","l1", "Length of the first arm");
    logger.save(l2,"l2","l2", "Length of the second arm");
    logger.save(Iz1,"Iz1","Iz1", "Inertia of the first arm");
    logger.save(m1,"m1","m1", "Mass of the first arm");
    logger.save(Iz2,"Iz2","Iz2", "Inertia of the second arm");
    logger.save(m2,"m2","m2", "Mass of the second arm");
    logger.save(m_effecteur,"m0","m0", "Mass of the effector");
    logger.save(c1,"c1","c1", "position of the center of mass on the first arm");
    logger.save(c2,"c2","c2", "position of the center of mass on the second arm");

    logger.setUnits("[m, m, kg.m^2, kg, kg.m^2, kg.m^2, kg, kg.m^2, m, m]");



    logger.update();


}
