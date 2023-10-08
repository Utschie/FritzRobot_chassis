#ifndef __EKF_H__
#define __EKF_H__
#include <Eigen/Dense>
using namespace Eigen;
#ifdef __cplusplus
extern "C" {
#endif


class EKF
{
	public:
		
		EKF();
	  EKF(float _q0, float _q1, float _q2,float _q3);
		float dt;//时间差
	  Matrix<float,4,1> state_vector;
	  void predict(float wx, float wy, float wz);
	  void update();
	  void setP(Matrix<float,4,4> P_);
	  void setQ(Matrix<float,4,4> Q_);
	  void setR(Matrix<float,3,3> R_);
	  void setz(float z1,float z2, float z3);
	
	private:
	  Matrix<float,4,4> P;//协方差矩阵6*6
	  Matrix<float,4,4> Q;
	  Matrix<float,3,3> R;
	  Matrix<float,4,1> quat_est;//4*1
	  Matrix<float,3,1> z;//用于保存来自加速度的的归一化的重力加速度值
	  
	  
};

#ifdef __cplusplus
}
#endif

#endif /* __EKF__ */