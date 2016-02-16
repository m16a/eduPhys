#ifndef _MY_UTILS_H_
#define _MY_UTILS_H_


Eigen::Matrix3f getCrossMatrix(Vector3f t)
{
	Eigen::Matrix3f res;
	res <<	0,	 -t(2),	 t(1),
    			t(2), 0, 	-t(0),
    			-t(1), t(0), 0;

    return res;
}


#endif// _MY_UTILS_H_