#include <iostream>
#include <Eigen/Geometry>

void euler_to_rot_mat(float roll, float pitch, float yaw, Eigen::Matrix3f &rot_mat)
{
	Eigen::AngleAxisf roll_angle(roll, Eigen::Vector3f::UnitZ());
	Eigen::AngleAxisf pitch_angle(pitch, Eigen::Vector3f::UnitY());
	Eigen::AngleAxisf yaw_angle(yaw, Eigen::Vector3f::UnitX());

	Eigen::Quaternionf q = roll_angle * pitch_angle * yaw_angle;
	rot_mat = q.matrix();
}

int main()
{
	Eigen::Matrix3f Ra;
	Eigen::Matrix3f Rb;
	float th_r_a = M_PI / 4, th_p_a = 0, th_y_a = M_PI/3;
	float th_r_b = -M_PI/3, th_p_b = 0, th_y_b = 0;
	euler_to_rot_mat(th_r_a, th_p_a, th_y_a,Ra);
	euler_to_rot_mat(th_r_b, th_p_b, th_y_b, Rb);

	std::cout << "Ra\n" << Ra << std::endl;
	std::cout << "Rb\n" << Rb << std::endl;

	Eigen::Matrix3f Rab;
	Eigen::Matrix3f Rba;
	Rab = Ra * Rb;
	Rba = Rb * Ra;

	std::cout << "Rab\n" << Rab << std::endl;
	std::cout << "Rba\n" << Rba << std::endl;

	Eigen::Quaternionf qa(Ra);
	Eigen::Quaternionf qb(Rb);
	Eigen::Vector4f qav;
	Eigen::Vector4f qbv;
	qav << qa.w(), qa.vec();
	qbv << qb.w(), qb.vec();
	// 
	std::cout << "norm qa " << qa.norm() << std::endl;
	std::cout << "norm qb " << qb.norm() << std::endl;
	std::cout << "qa\n" << qav << std::endl;
	std::cout << "qb\n" << qbv << std::endl;
	// std::cout << "qb\n" << qb << std::endl;

	Eigen::Quaternionf qc;
	Eigen::Quaternionf qd;
	qc = qa * qb;
	qd = qb * qa;
	std::cout << "qc\n" << qc.w() << "\n" << qc.vec() << std::endl;
	std::cout << "qd\n" << qd.w() << "\n" << qd.vec() << std::endl;

	Eigen::Quaternionf qe;
	Eigen::Quaternionf qf;
	qe = qa * qb.inverse();
	qf = qe * qb;

	std::cout << "qe\n" << qe.w() << "\n" << qe.vec() << std::endl;
	std::cout << "qf\n" << qf.w() << "\n" << qf.vec() << std::endl;

}
