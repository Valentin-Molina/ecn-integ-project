Ce package (suivi_traj_ek) contient 2 noeuds independants qui implementent une loi de controle pour suivre une trajectoire.
Selon la loi de controle qu'on veut utiliser, on fait tourner Commande_PID ou Commande_CTC.
Ces noeuds fonctionnent en parallele avec un generateur de trajectoire, un modele de robot et dans le cas de CTC un serveur de service du modele dynamique.
Les noeuds recuperent la trajectoire desiree sur le topic "/Trajectoire".
Ils recuperent l'etat actuel du robot sur le topic "/joint_states".
Ils publient la commande en couple des moteurs du robot sur 2 topics "/hand_effort_controller/command" pour q1 et "/finger_effort_controller/command" pour q2.
Dans le cas de Commande_CTC on fait appel a un service qui applique le modele dynamique inverse (calcul des couples moteur en fonction de la position, vitesse et acceleration des vitesses articulaires).

Pour faire tourner un noeud:
rosrun suivi_traj_ek Commande_CTC
ou
rosrun suivi_traj_ek Commande_PID
