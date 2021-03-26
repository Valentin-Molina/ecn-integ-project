#include <calc.h>

vpMatrix GetRbc(tf::TransformListener &listener)
{
	vpMatrix Rbc(6, 6);
	tf::StampedTransform transform;
	listener.waitForTransform("/camera_link", "/world", ros::Time(0), ros::Duration(4.0));
	listener.lookupTransform("/camera_link", "/world", ros::Time(0), transform);
	for (int i = 0; i < 3; i++)
		for (int j = 0; j < 3; j++)
		{
			Rbc[i][j] = transform.getBasis()[i][j];
			Rbc[i + 3][j + 3] = Rbc[i][j];
		}
	return Rbc;
}

vpMatrix GetJac(double q1, double q2, double l1, double l2)
{
	vpMatrix Jac(6, 2);

	double s1 = sin(q1);
	double s2 = sin(q2);
	double c1 = cos(q1);
	double c2 = cos(q2);
	double s12 = sin(q1 + q2);
	double c12 = cos(q1 + q2);

	Jac[0][0] = -1 * l1 * s1 - l2 * s12;
	Jac[0][1] = -1 * l2 * s12;
	Jac[1][0] = l1 * c1 - l2 * c12;
	Jac[1][1] = l2 * c12;
	Jac[5][0] = 1;
	Jac[5][1] = 1;

	return Jac;
}

using namespace std;