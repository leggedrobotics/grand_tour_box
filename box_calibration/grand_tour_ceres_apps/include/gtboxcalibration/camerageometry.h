//
// Created by fu on 08/08/2024.
//

#ifndef GRAND_TOUR_CERES_APPS_CAMERA_GEOMETRY_H
#define GRAND_TOUR_CERES_APPS_CAMERA_GEOMETRY_H

#include <iostream>
#include <type_traits>
#include <Eigen/Dense>
#include <ceres/ceres.h>
#include <ceres/rotation.h>


template<typename T, typename = void>
struct has_static_n_parameters : std::false_type {
};

template<typename T>
struct has_static_n_parameters<T, std::void_t<decltype(T::NUM_PARAMETERS)>> : std::true_type {
};

template<class DerivedParameterClass>
struct CeresOperationView {
    static constexpr int nParameters() {
        static_assert(has_static_n_parameters<DerivedParameterClass>::value,
                      "The new class you are implementing derives from CeresParameterBlockAdaptor"
                      " and must define a field called: static constexpr int NUM_PARAMETERS");
        return DerivedParameterClass::NUM_PARAMETERS;
    }

    virtual bool handleSetParameterization(ceres::Problem &problem, double *parameters) = 0;

    virtual void print(const double *parameters) = 0;
};

struct Distortion : CeresOperationView<Distortion> {
    static constexpr int NUM_PARAMETERS = 4;
    enum Type {
        Fisheye = 1,
        RadTan = 2
    };
};

struct FisheyeDistortion : public Distortion {

    void print(const double *const parameters) override {
        std::cout << "k1: " << parameters[0]
                  << "\tk2: " << parameters[1]
                  << "\tk3: " << parameters[2]
                  << "\tk4: " << parameters[3] << std::endl;
    }

    bool handleSetParameterization(ceres::Problem &problem, double *parameters) override {
        return true;
    }

    template<typename T>
    Eigen::Matrix<T, 2, Eigen::Dynamic> operator()(const T *const parameters, Eigen::Matrix<T, 2, Eigen::Dynamic> y)
    const {
        Eigen::Matrix<T, 1, Eigen::Dynamic> r, theta, theta2, theta4, theta6, theta8, thetad, scaling;
        r = (y.row(0).array() * y.row(0).array() + y.row(1).array() * y.row(1).array()).sqrt();
        theta = r.array().atan();
        theta2 = theta.array() * theta.array();
        theta4 = theta2.array() * theta2.array();
        theta6 = theta4.array() * theta2.array();
        theta8 = theta4.array() * theta4.array();

        T k1 = T(parameters[0]);
        T k2 = T(parameters[1]);
        T k3 = T(parameters[2]);
        T k4 = T(parameters[3]);

        thetad = theta.array()
                 * (T(1.0) + k1 * theta2.array() + k2 * theta4.array() + k3 * theta6.array() + k4 * theta8.array());
        scaling = (r.array() > T(1e-8)).select(thetad.array() / r.array(), T(1.0));

        y.row(0).array() *= scaling.array();
        y.row(1).array() *= scaling.array();
        return y;
    }

    double parameters_[NUM_PARAMETERS]{};
};

struct RadTanDistortion : public Distortion {
    void print(const double *parameters) override {
        std::cout << "k1: " << parameters[0]
                  << "\tk2: " << parameters[1]
                  << "\tp1: " << parameters[2]
                  << "\tp2: " << parameters[3] << std::endl;
    }

    template<typename T>
    Eigen::Matrix<T, 2, Eigen::Dynamic> operator()(const T *const parameters, Eigen::Matrix<T, 2, Eigen::Dynamic> y)
    const {

        Eigen::Matrix<T, 1, Eigen::Dynamic> mx2_u, my2_u, mxy_u, rho2_u, rad_dist_u;
        T k1 = T(parameters[0]);
        T k2 = T(parameters[1]);
        T p1 = T(parameters[2]);
        T p2 = T(parameters[3]);

        mx2_u = y.row(0).array() * y.row(0).array();
        my2_u = y.row(1).array() * y.row(1).array();
        mxy_u = y.row(0).array() * y.row(1).array();
        rho2_u = mx2_u.array() + my2_u.array();
        rad_dist_u = k1 * rho2_u.array() + k2 * rho2_u.array() * rho2_u.array();

        y.row(0).array() += y.row(0).array() * rad_dist_u.array() + T(2.0) * p1 * mxy_u.array() +
                            p2 * (rho2_u.array() + T(2.0) * mx2_u.array());
        y.row(1).array() += y.row(1).array() * rad_dist_u.array() + T(2.0) * p2 * mxy_u.array() +
                            p1 * (rho2_u.array() + T(2.0) * my2_u.array());
        return y;
    }

    bool handleSetParameterization(ceres::Problem &problem, double *parameters) override {
        return true;
    }

    double parameters_[NUM_PARAMETERS]{};
};


struct PinholeProjection : CeresOperationView<PinholeProjection> {
    PinholeProjection(int width, int height) : width_{width}, height_{height} {}

    bool handleSetParameterization(ceres::Problem &problem, double *parameters) override {
        // The image width and height aren't mapped to the ceres problem
        return true;
    }

    template<typename T>
    Eigen::Matrix<T, 2, Eigen::Dynamic>
    operator()(const T *const parameters, Eigen::Matrix<T, 2, Eigen::Dynamic> y) const {
        y.row(0).array() *= T(parameters[0]);
        y.row(0).array() += T(parameters[2]);
        y.row(1).array() *= T(parameters[1]);
        y.row(1).array() += T(parameters[3]);
        return y;
    }

    void print(const double *const parameters) override {
        std::cout << "fx: " << parameters[0] << "\t"
                  << "fy: " << parameters[1] << "\t"
                  << "cx: " << parameters[2] << "\t"
                  << "cy: " << parameters[3] << "\t"
                  << "width: " << width_ << "\t"
                  << "height: " << height_ << "\t"
                  << std::endl;
    }

    static constexpr int NUM_INTRINSIC_PARAMETERS = 4;
    static constexpr int NUM_PARAMETERS = NUM_INTRINSIC_PARAMETERS;
    const int width_, height_;
    double parameters_[NUM_PARAMETERS]{};
};


struct SE3Transform : CeresOperationView<SE3Transform> {
    bool handleSetParameterization(ceres::Problem &problem, double *parameters) override {
        ceres::Manifold *manifold = new ceres::ProductManifold<ceres::QuaternionManifold,
                ceres::EuclideanManifold<3>>();
        problem.SetManifold(parameters, manifold);
        return true;
    }

    template<typename T>
    Eigen::Matrix<T, 3, Eigen::Dynamic>
    operator()(const T *const parameters, const Eigen::Matrix<T, 3, Eigen::Dynamic> &points_in_source) const {
        Eigen::Matrix<T, 3, 3> rotation = Eigen::Matrix<T, 3, 3>::Identity();
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(parameters + NUM_QUATERNION_PARAMS);

        // Convert quaternion to rotation matrix
        ceres::QuaternionToRotation(parameters, ceres::ColumnMajorAdapter3x3(rotation.data()));

        return (rotation * points_in_source).colwise() + translation;
    }

    template<typename T>
    Eigen::Matrix<T, 3, Eigen::Dynamic>
    inverseTransform(const T *const parameters, const Eigen::Matrix<T, 3, Eigen::Dynamic> &points_in_source) const {
        Eigen::Matrix<T, 3, 3> rotation = Eigen::Matrix<T, 3, 3>::Identity();
        Eigen::Map<const Eigen::Matrix<T, 3, 1>> translation(parameters + NUM_QUATERNION_PARAMS);

        // Convert quaternion to rotation matrix
        ceres::QuaternionToRotation(parameters, ceres::ColumnMajorAdapter3x3(rotation.data()));

        return (rotation.transpose() * points_in_source).colwise() - (rotation.transpose() * translation);
    }

    void print(const double *const parameters) override {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        ceres::QuaternionToRotation(parameters, ceres::ColumnMajorAdapter3x3(rotation.data()));
        std::cout << rotation << std::endl;
        std::cout << Eigen::Map<const Eigen::Vector3d>(parameters + NUM_QUATERNION_PARAMS) << std::endl;
    }

    static Eigen::Matrix4d toEigen(const double *const parameters) {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        ceres::QuaternionToRotation(parameters, ceres::ColumnMajorAdapter3x3(rotation.data()));
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = rotation;
        T.block<3, 1>(0, 3) = Eigen::Map<const Eigen::Vector3d>(parameters + NUM_QUATERNION_PARAMS);
        return T;
    }

    static Eigen::Affine3d toEigenAffine(const double *const parameters) {
        Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
        ceres::QuaternionToRotation(parameters, ceres::ColumnMajorAdapter3x3(rotation.data()));
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.linear() = rotation;
        T.translation() = Eigen::Map<const Eigen::Vector3d>(parameters + NUM_QUATERNION_PARAMS);
        return T;
    }

    std::vector<std::vector<double>> toVector(const double *const parameters) {
        const Eigen::Matrix4d T = this->toEigen(parameters);
        return {{T(0, 0), T(0, 1), T(0, 2), T(0, 3)},
                {T(1, 0), T(1, 1), T(1, 2), T(1, 3)},
                {T(2, 0), T(2, 1), T(2, 2), T(2, 3)},
                {T(3, 0), T(3, 1), T(3, 2), T(3, 3)}};
    }

    static void assignToData(const Eigen::Affine3d &input_transform, double *parameters) {
        const Eigen::Matrix3d rotation = input_transform.rotation();
        ceres::RotationMatrixToQuaternion(ceres::ColumnMajorAdapter3x3(rotation.data()), parameters);
        std::copy(input_transform.translation().data(), input_transform.translation().data() + NUM_EUCLIDEAN_PARAMS,
                  parameters + NUM_QUATERNION_PARAMS);
    }

    static constexpr int NUM_QUATERNION_PARAMS = 4;
    static constexpr int NUM_EUCLIDEAN_PARAMS = 3;
    static constexpr int NUM_PARAMETERS = NUM_QUATERNION_PARAMS + NUM_EUCLIDEAN_PARAMS;
    double parameters_[NUM_PARAMETERS]{};
};

template<typename T>
Eigen::Matrix<T, 2, Eigen::Dynamic> getNormalised(Eigen::Matrix<T, 3, Eigen::Dynamic> points_in_camera) {
    Eigen::Matrix<T, 1, Eigen::Dynamic> depths = (points_in_camera.row(2).array() > T(1e-4)).select(
            points_in_camera.row(2).array(), T(1.0));
    Eigen::Matrix<T, 2, Eigen::Dynamic> normalised_points = (points_in_camera.array().rowwise() /
                                                             depths.array()).topRows(2);
    return normalised_points;
}

struct PinholeCamera {
    PinholeCamera(PinholeProjection pinholeProjection, Distortion::Type distortionType) :
            pinholeProjection(std::move(pinholeProjection)),
            distortion_type_(distortionType) {}

    template<typename T>
    Eigen::Matrix<T, 2, Eigen::Dynamic>
    project(const T *const projection_parameters, const T *const distortion_parameters,
            const Eigen::Matrix<T, 3, Eigen::Dynamic> &points_in_camera) const {
        Eigen::Matrix<T, 2, Eigen::Dynamic> normalised_points = getNormalised(points_in_camera);
        Eigen::Matrix<T, 2, Eigen::Dynamic> distorted_points;
        if (distortion_type_ == Distortion::Fisheye) {
            distorted_points = fisheye_distortion_(distortion_parameters, normalised_points);
        } else if (distortion_type_ == Distortion::RadTan) {
            distorted_points = radtan_distortion_(distortion_parameters, normalised_points);
        } else {
            std::cerr << "Unmodelled distortion" << std::endl;
        }
        Eigen::Matrix<T, 2, Eigen::Dynamic> projected_coordinates = pinholeProjection(projection_parameters,
                                                                               distorted_points);
        return projected_coordinates;
    }

    template<typename T>
    Eigen::Matrix<T, 2, Eigen::Dynamic>
    transformToCameraThenProject(const T *const T_camera_modelpoints_parameters,
                                 const T *const projection_parameters, const T *const distortion_parameters,
                                 const Eigen::Matrix<T, 3, Eigen::Dynamic> &points_in_model) const {
        Eigen::Matrix<T, 3, Eigen::Dynamic> points_in_camera = se3_transform(T_camera_modelpoints_parameters,
                                                                             points_in_model);
        return this->template project(projection_parameters, distortion_parameters,
                points_in_camera);
    }

    static constexpr int NUM_PARAMETERS = SE3Transform::NUM_PARAMETERS +
            Distortion::NUM_PARAMETERS + PinholeProjection::NUM_PARAMETERS;
    Distortion::Type distortion_type_;
    SE3Transform se3_transform;
    PinholeProjection pinholeProjection;
    FisheyeDistortion fisheye_distortion_;
    RadTanDistortion radtan_distortion_;
};


#endif //GRAND_TOUR_CERES_APPS_CAMERA_GEOMETRY_H
