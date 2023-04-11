#ifndef PATH_UTILS_H_
#define PATH_UTILS_H_

#include <Eigen/Dense>

namespace path {

double interpolate(double start, double end, double ratio)
{
    return start + (end - start) * ratio;
}

/**
 * Generates a simple straight line path with n number of points.
*/
template<int Size>
Eigen::Matrix<double, Size, 3> generateSimplePath(const Eigen::Vector<double, 3>& startPosition,
                                                  const Eigen::Vector<double, 3>& endPosition)
{
    Eigen::Matrix<double, Size, 3> path = Eigen::Matrix<double, Size, 3>::Zero();
    
    for (unsigned int i = 0; i < Size; i++) {
        const double ratio = static_cast<double>(i) / static_cast<double>((Size - 1));

        path(i, 0) = interpolate(startPosition(0), endPosition(0), ratio);
        path(i, 1) = interpolate(startPosition(1), endPosition(1), ratio);
        path(i, 2) = interpolate(startPosition(2), endPosition(2), ratio);
    }

    return path;
}

} // namespace path

#endif // PATH_UTILS_H_
