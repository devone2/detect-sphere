#ifndef FKRESIDUAL_H
#define FKRESIDUAL_H

#include "ceres/ceres.h"

#include <math.h>

using ceres::AutoDiffCostFunction;
using ceres::CostFunction;
using ceres::Problem;
using ceres::Solver;
using ceres::Solve;

template <typename T>
struct LinkParam {
    Eigen::Transform<T,3,Eigen::Affine> origin;
    Eigen::Matrix<T, 3, 1> rotationVector;

    /*
    LinkParam(Eigen::Translation<double, 3> originTranslation, Eigen::AngleAxisd originRotation, Eigen::Vectord3d rotationVector) {
        this->origin = originTranslation * originRotation;
        this->rotationVector = rotationVector;
    }*/
};

struct Observation {
    Eigen::Matrix<double, 3, 1> obsPos;
    std::vector<double> joints;
};

struct FKResidual {
    FKResidual(Observation obs1, Observation obs2) {
        _obs1 = obs1;
        _obs2 = obs2;
    }

  template <typename T> bool operator()(const T* const camera, const T* const mods, T* residual) const {
    typedef Eigen::Matrix<T, 3, 1> Vec3;

    Eigen::Transform<T,3,Eigen::Affine> G = Eigen::Translation<T, 3>(camera[0],camera[1],camera[2]) * Eigen::AngleAxis<T>(camera[3], Vec3(camera[4],camera[5], camera[6]));
    Vec3 P1 = convertToVec3<T>(_obs1.obsPos);
    Vec3 P2 = convertToVec3<T>(_obs2.obsPos);

    std::vector<LinkParam<T>> modLinks = makeModLinks<T>(mods);
    Eigen::Transform<T,3,Eigen::Affine> H1 = calculateFK<T>(modLinks, _obs1.joints);
    Eigen::Transform<T,3,Eigen::Affine> H2 = calculateFK<T>(modLinks, _obs2.joints);

    Vec3 L1 = H2 * H1.inverse() * G * P1;
    Vec3 L2 = G * P2;

    Vec3 result = L1 - L2;

    residual[0] = result(0);
    residual[1] = result(1);
    residual[2] = result(2);

    return true;
  }

    template <typename T> Eigen::Matrix<T, 3, 1> convertToVec3(Eigen::Matrix<double, 3, 1> pos) const {
        return Eigen::Matrix<T, 3, 1>(T(pos(0)),T(pos(1)),T(pos(2)));
    }

    template <typename T> Eigen::Transform<T,3,Eigen::Affine> static calculateFK(const std::vector<LinkParam<T>>& links, const std::vector<double>& joints) {
        Eigen::Transform<T,3,Eigen::Affine> pose = Eigen::Transform<T,3,Eigen::Affine>::Identity();

        for(auto i=0; i<links.size();i++) {
           pose = pose * links[i].origin * Eigen::AngleAxis<T>(T(joints[i]), links[i].rotationVector);
        }

        return pose;
    }

    template <typename T> std::vector<LinkParam<T>> static makeModLinks(const T * const linkModifications) {

        Eigen::Matrix<T, 3, 1> unitZ(T(0),T(0),T(1));
        Eigen::Matrix<T, 3, 1> unitX(T(1),T(0),T(0));
        Eigen::Matrix<T, 3, 1> zero(T(0),T(0),T(0));

        std::vector<LinkParam<T>> links;
        links.push_back( {Eigen::Translation<T, 3>(T(0),T(0),T(0.333) * linkModifications[0]) * Eigen::AngleAxis<T>::Identity(), unitZ});
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(0),T(0)) * Eigen::AngleAxis<T>( T(-M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(-0.316) * linkModifications[2],T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0.0825) * linkModifications[3],T(0),T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(-0.0825),T(0.384) * linkModifications[4],T(0)) * Eigen::AngleAxis<T>( T(-M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(0),T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0.088) * linkModifications[6],T(0),T(0)) * Eigen::AngleAxis<T>( T(M_PI/2), unitX), unitZ });
        links.push_back({ Eigen::Translation<T, 3>(T(0),T(0),T(0.107) * linkModifications[7]) * Eigen::AngleAxis<T>::Identity(), zero });

        return links;
    }

  private:
    Observation _obs1;
    Observation _obs2;
};


#endif // FKRESIDUAL_H
