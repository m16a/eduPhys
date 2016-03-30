#include "my_utils.h"

Eigen::Matrix3f getCrossMatrix(Vector3f t)
{
	Eigen::Matrix3f res;
	res <<	0,	 -t(2),	 t(1),
    			t(2), 0, 	-t(0),
    			-t(1), t(0), 0;

    return res;
}

QDebug operator<<(QDebug dbg, const Vector3f& v)
{
    dbg.nospace() << "(" << v.x() << ", " << v.y() <<  ", " << v.z() << ")";

    return dbg.space();
}

