Utilisation du package gkd_models:

Il est dans un premier temps nécessaire de rendre les scripts executables, pour cela :
- Ouvrir l'explorateur de fichiers
- Aller dans scripts
- Selectionner le fichier gkd.py
- clic droit
- Propriétés 
- Permissions
- Cocher "allow this file to run as a programm"

Pour utilisier les services, il est nécessaire de lancer le noeud gkd.py
rosrun gkd_models gkd.py          dans la console, ou via un launch file

Les services peuvent alors être appelés à l'intérieur d'un autre noeud par les lignes suivantes:

- from gkd_models.srv import * #en début de fichier (en pyhton, même principe en C++)
- rospy.wait_for_service('Dynamic')
	dyna = rospy.ServiceProxy('Dynamic', Dynamic)  #pour appeler le service et réccupérer sa réponse quand celui-ci est disponible
                                                 #ici la variable dyna représente le service
                                                 
Pour réccupérer la réponse du service, idéalement :

try:
		response = dyna(input)
except rospy.ServiceException as exc:
		print("Service did not process request: " + str(exc))
  
Où response est le résultat du calcul par le service, et input est l'entrée de service.



Les entrées et sorties des services que nous proposons sont les suivantes:

Dynamic :
E : JointState
S : JointState

Kinematic :
E : JointState
S : float[2] # tableau de 2 réels : vitesse dans l'espace cartésien

MGD :
E : JointState
S : float[2]  # tableau de 2 réels : position dans l'espace cartésien

MGI : 
E : - float[2] # position dans l'espace cartésien
    - bool # high_elbow : position du coude pour discriminer les deux solutions du mgi
S : JointState
