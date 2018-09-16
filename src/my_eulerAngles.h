#ifndef _MY_EULERANGELS_H_
#define _MY_EULERANGELS_H_

#include <Eigen/Geometry>

using namespace Eigen;

template<typename _Scalar> class EulerAngles
{
public:
  enum { Dim = 3 };
  typedef _Scalar Scalar;
  typedef Matrix<Scalar,3,3> Matrix3;
  typedef Matrix<Scalar,3,1> Vector3;
  typedef Quaternion<Scalar> QuaternionType;

protected:

  Vector3 m_angles;

public:

  EulerAngles() {}
  inline EulerAngles(Scalar a0, Scalar a1, Scalar a2) : m_angles(a0, a1, a2) {}
  inline EulerAngles(const QuaternionType& q) { *this = q; }

  const Vector3& coeffs() const { return m_angles; }
  Vector3& coeffs() { return m_angles; }

  EulerAngles& operator=(const QuaternionType& q)
  {
    Matrix3 m = q.toRotationMatrix();
    return *this = m;
  }

  EulerAngles& operator=(const Matrix3& m)
  {
    // mat =  cy*cz          -cy*sz           sy
    //        cz*sx*sy+cx*sz  cx*cz-sx*sy*sz -cy*sx
    //       -cx*cz*sy+sx*sz  cz*sx+cx*sy*sz  cx*cy
    m_angles.coeffRef(1) = std::asin(m.coeff(0,2));
    m_angles.coeffRef(0) = std::atan2(-m.coeff(1,2),m.coeff(2,2));
    m_angles.coeffRef(2) = std::atan2(-m.coeff(0,1),m.coeff(0,0));
    return *this;
  }

  Matrix3 toRotationMatrix(void) const
  {
    Vector3 c = m_angles.array().cos();
    Vector3 s = m_angles.array().sin();
    Matrix3 res;
    res <<  c.y()*c.z(),                    -c.y()*s.z(),                   s.y(),
            c.z()*s.x()*s.y()+c.x()*s.z(),  c.x()*c.z()-s.x()*s.y()*s.z(),  -c.y()*s.x(),
            -c.x()*c.z()*s.y()+s.x()*s.z(), c.z()*s.x()+c.x()*s.y()*s.z(),  c.x()*c.y();
    return res;
  }

  operator QuaternionType() { return QuaternionType(toRotationMatrix()); }
};



#endif// _MY_EULERANGELS_H_

