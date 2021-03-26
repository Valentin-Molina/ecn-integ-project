# example_gazebo_set_state

This node assumes the model "cible_simple" has been loaded into Gazebo.  
Running this node performs a single service call to /gazebo/set_model_state.
The service call causes the model "cible_simple" to
be assigned with a specified pose.  

#comment lancer le modele de la cible

L'urdf de la cible se trouve dans le package "integ_description"
La cible se lance avec le launchfile du robot, "arm.launch" dans le package "integ_description"

#modifications

Il est possible de modifier la trajectoire de la cible en modifiant l'amplitude et la fréquence de la sinusoide dans le node en changeant les valeurs de "model_state_srv_msg.request.model_state.pose.position"

    
