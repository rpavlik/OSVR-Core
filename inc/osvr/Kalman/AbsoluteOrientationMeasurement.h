/** @file
    @brief Header for measurements of absolute orientation.

    @date 2015

    @author
    Sensics, Inc.
    <http://sensics.com/osvr>
*/

// Copyright 2015 Sensics, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//        http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INCLUDED_AbsoluteOrientationMeasurement_h_GUID_71285DD8_A6F1_47A8_4B2E_B10171C91248
#define INCLUDED_AbsoluteOrientationMeasurement_h_GUID_71285DD8_A6F1_47A8_4B2E_B10171C91248

// Internal Includes
#include "FlexibleKalmanBase.h"
#include "PoseState.h"
#include "ExternalQuaternion.h"
#include <osvr/Util/EigenCoreGeometry.h>

// Library/third-party includes
// - none

// Standard includes
// - none

namespace osvr {
namespace kalman {
    /// The measurement here has been split into a base and derived type, so
    /// that the derived type only contains the little bit that depends on a
    /// particular state type.
    class AbsoluteOrientationBase {
      public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        static const types::DimensionType DIMENSION = 4;
        using MeasurementVector = types::Vector<DIMENSION>;
        using MeasurementSquareMatrix = types::SquareMatrix<DIMENSION>;
        AbsoluteOrientationBase(Eigen::Quaterniond const &quat,
                                types::Vector<3> const &exponentialVariance)
            : m_measurement(quat),
              m_covariance(MeasurementSquareMatrix::Identity()),
            m_exponentialVariance(exponentialVariance) {

        }

        template <typename State>
        MeasurementSquareMatrix const &getCovariance(State const &s) {
            // compute the ln of the measurement.
            auto vecnorm = m_measurement.vec().norm();
            auto phi = std::atan2(vecnorm, m_measurement.w());
            auto phiOverNorm = vecnorm < 1e-4? (phi/std::sin(phi)) : phi/vecnorm;
            Eigen::Vector3d qLog = m_measurement.vec() * phiOverNorm;

            /// The covariance in Quat space is G C G^T, where G is the Jacobian
            /// of a quaternion with respect to exponential coordinates (evaluated at the
            /// log of the measurement in getCovariance) and C is the covariance in
            /// exponential coordinates space.
            Eigen::Matrix<double, 4, 3> G = external_quat::jacobian(qLog);

            m_covariance = G * m_exponentialVariance.asDiagonal() * G.transpose();
            return m_covariance;
        }

        /// Gets the measurement residual, also known as innovation: predicts
        /// the measurement from the predicted state, and returns the
        /// difference.
        ///
        /// State type doesn't matter as long as we can getCombinedQuaternion()
        template <typename State>
        MeasurementVector getResidual(State const &s) const {
            const Eigen::Quaterniond prediction = s.getCombinedQuaternion();
            const Eigen::Quaterniond residual =
                m_measurement * prediction.conjugate();
            return residual.coeffs();
        }
        /// Convenience method to be able to store and re-use measurements.
        void setMeasurement(Eigen::Quaterniond const &quat) {
            m_measurement = quat;
        }

      private:
        Eigen::Quaterniond m_measurement;
        MeasurementSquareMatrix m_covariance;
        types::Vector<3> m_exponentialVariance;
    };

    /// This is the subclass of AbsoluteOrientationBase: only explicit
    /// specializations, and on state types.
    template <typename StateType> class AbsoluteOrientationMeasurement;

    /// AbsoluteOrientationMeasurement with a pose_externalized_rotation::State
    template <>
    class AbsoluteOrientationMeasurement<pose_externalized_rotation::State>
        : public AbsoluteOrientationBase {
      public:
        using State = pose_externalized_rotation::State;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        static const types::DimensionType STATE_DIMENSION =
            types::Dimension<State>::value;
        using Base = AbsoluteOrientationBase;

        AbsoluteOrientationMeasurement(Eigen::Quaterniond const &quat,
                                       types::Vector<3> const &eulerVariance)
            : Base(quat, eulerVariance) {}

        types::Matrix<DIMENSION, STATE_DIMENSION>
        getJacobian(State const &s) const {
            using namespace pose_externalized_rotation;
            using Jacobian = types::Matrix<DIMENSION, STATE_DIMENSION>;
            Jacobian ret = Jacobian::Zero();
            ret.block<DIMENSION, 3>(0, 3) = external_quat::jacobian(
                incrementalOrientation(s.stateVector()));
            return ret;
        }
    };
} // namespace kalman
} // namespace osvr
#endif // INCLUDED_AbsoluteOrientationMeasurement_h_GUID_71285DD8_A6F1_47A8_4B2E_B10171C91248
