/**
 * @file FusionAhrs.c
 * @author Seb Madgwick
 * @brief The AHRS sensor fusion algorithm to combines gyroscope, accelerometer,
 * and magnetometer measurements into a single measurement of orientation
 * relative to the Earth (NWU convention).
 *
 * The algorithm behaviour is governed by a gain.  A low gain will decrease the
 * influence of the accelerometer and magnetometer so that the algorithm will
 * better reject disturbances causes by translational motion and temporary
 * magnetic distortions.  However, a low gain will also increase the risk of
 * drift due to gyroscope calibration errors.  A typical gain value suitable for
 * most applications is 0.5.
 *
 * The algorithm allows the application to define a minimum and maximum valid
 * magnetic field magnitude.  The algorithm will ignore magnetic measurements
 * that fall outside of this range.  This allows the algorithm to reject
 * magnetic measurements that do not represent the direction of magnetic North.
 * The typical magnitude of the Earth's magnetic field is between 20 uT and
 * 70 uT.
 *
 * The algorithm can be used without a magnetometer.  Measurements of
 * orientation obtained using only gyroscope and accelerometer measurements
 * can be expected to drift in the yaw component of orientation only.  The
 * application can reset the drift in yaw by setting the yaw to a specified
 * angle at any time.
 *
 * The algorithm provides the measurement of orientation as a quaternion.  The
 * library includes functions for converting this quaternion to a rotation
 * matrix and Euler angles.
 *
 * The algorithm also provides a measurement of linear acceleration and Earth
 * acceleration.  Linear acceleration is equal to the accelerometer  measurement
 * with the 1 g of gravity removed.  Earth acceleration is a measurement of
 * linear acceleration in the Earth coordinate frame.
 */

//------------------------------------------------------------------------------
// Includes

#include "FusionAhrs.h"
#include <float.h> // FLT_MAX
#include <math.h> // atan2f, cosf, sinf

//------------------------------------------------------------------------------
// Definitions

/**
 * @brief Initial gain used during the initialisation period.  The gain used by
 * each algorithm iteration will ramp down from this initial gain to the
 * specified algorithm gain over the initialisation period.
 */
#define INITIAL_GAIN (10.0f)

/**
 * @brief Initialisation period (in seconds).
 */
#define INITIALISATION_PERIOD (3.0f)

//------------------------------------------------------------------------------
// Functions

/**
 * @brief Initialises the AHRS algorithm structure.
 * @param fusionAhrs AHRS algorithm structure.
 * @param gain AHRS algorithm gain.
 */
void FusionAhrsInitialise(FusionAhrs * const fusionAhrs, const float gain) {
    fusionAhrs->gain = gain;
    fusionAhrs->minimumMagneticFieldSquared = 0.0f;
    fusionAhrs->maximumMagneticFieldSquared = FLT_MAX;
    fusionAhrs->quaternion = FUSION_QUATERNION_IDENTITY;
    fusionAhrs->linearAcceleration = FUSION_VECTOR3_ZERO;
    fusionAhrs->rampedGain = INITIAL_GAIN;
    fusionAhrs->zeroYawPending = false;
}

/**
 * @brief Sets the AHRS algorithm gain.  The gain must be equal or greater than
 * zero.
 * @param gain AHRS algorithm gain.
 */
void FusionAhrsSetGain(FusionAhrs * const fusionAhrs, const float gain) {
    fusionAhrs->gain = gain;
}

/**
 * @brief Sets the minimum and maximum valid magnetic field magnitudes in uT.
 * @param fusionAhrs AHRS algorithm structure.
 * @param minimumMagneticField Minimum valid magnetic field magnitude.
 * @param maximumMagneticField Maximum valid magnetic field magnitude.
 */
void FusionAhrsSetMagneticField(FusionAhrs * const fusionAhrs, const float minimumMagneticField, const float maximumMagneticField) {
    fusionAhrs->minimumMagneticFieldSquared = minimumMagneticField * minimumMagneticField;
    fusionAhrs->maximumMagneticFieldSquared = maximumMagneticField * maximumMagneticField;
}

/**
 * @brief Updates the AHRS algorithm.  This function should be called for each
 * new gyroscope measurement.
 * @param fusionAhrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param magnetometer Magnetometer measurement in uT.
 * @param samplePeriod Sample period in seconds.  This is the difference in time
 * between the current and previous gyroscope measurements.
 */
void FusionAhrsUpdate(FusionAhrs * const fusionAhrs, const FusionVector3 gyroscope, const FusionVector3 accelerometer, const FusionVector3 magnetometer, const float samplePeriod) {
#define Q fusionAhrs->quaternion.element // define shorthand label for more readable code

    // Calculate feedback error
    FusionVector3 halfFeedbackError = FUSION_VECTOR3_ZERO; // scaled by 0.5 to avoid repeated multiplications by 2
    do {
        // Abandon feedback calculation if accelerometer measurement invalid
        if ((accelerometer.axis.x == 0.0f) && (accelerometer.axis.y == 0.0f) && (accelerometer.axis.z == 0.0f)) {
            break;
        }

        // Calculate direction of gravity assumed by quaternion
        const FusionVector3 halfGravity = {
            .axis.x = Q.x * Q.z - Q.w * Q.y,
            .axis.y = Q.w * Q.x + Q.y * Q.z,
            .axis.z = Q.w * Q.w - 0.5f + Q.z * Q.z,
        }; // equal to 3rd column of rotation matrix representation scaled by 0.5

        // Calculate accelerometer feedback error
        halfFeedbackError = FusionVectorCrossProduct(FusionVectorFastNormalise(accelerometer), halfGravity);

        // Abandon magnetometer feedback calculation if magnetometer measurement invalid
        const float magnetometerMagnitudeSquared = FusionVectorMagnitudeSquared(magnetometer);
        if ((magnetometerMagnitudeSquared < fusionAhrs->minimumMagneticFieldSquared) || (magnetometerMagnitudeSquared > fusionAhrs->maximumMagneticFieldSquared)) {
            break;
        }

        // Compute direction of 'magnetic west' assumed by quaternion
        const FusionVector3 halfWest = {
            .axis.x = Q.x * Q.y + Q.w * Q.z,
            .axis.y = Q.w * Q.w - 0.5f + Q.y * Q.y,
            .axis.z = Q.y * Q.z - Q.w * Q.x
        }; // equal to 2nd column of rotation matrix representation scaled by 0.5

        // Calculate magnetometer feedback error
        halfFeedbackError = FusionVectorAdd(halfFeedbackError, FusionVectorCrossProduct(FusionVectorFastNormalise(FusionVectorCrossProduct(accelerometer, magnetometer)), halfWest));

    } while (false);

    // Ramp down gain until initialisation complete
    if (fusionAhrs->gain == 0) {
        fusionAhrs->rampedGain = 0; // skip initialisation if gain is zero
    }
    float feedbackGain = fusionAhrs->gain;
    if (fusionAhrs->rampedGain > fusionAhrs->gain) {
        fusionAhrs->rampedGain -= (INITIAL_GAIN - fusionAhrs->gain) * samplePeriod / INITIALISATION_PERIOD;
        feedbackGain = fusionAhrs->rampedGain;
    }

    // Convert gyroscope to radians per second scaled by 0.5
    FusionVector3 halfGyroscope = FusionVectorMultiplyScalar(gyroscope, 0.5f * FusionDegreesToRadians(1.0f));

    // Apply feedback to gyroscope
    halfGyroscope = FusionVectorAdd(halfGyroscope, FusionVectorMultiplyScalar(halfFeedbackError, feedbackGain));

    // Integrate rate of change of quaternion
    fusionAhrs->quaternion = FusionQuaternionAdd(fusionAhrs->quaternion, FusionQuaternionMultiplyVector(fusionAhrs->quaternion, FusionVectorMultiplyScalar(halfGyroscope, samplePeriod)));

    // Normalise quaternion
    fusionAhrs->quaternion = FusionQuaternionFastNormalise(fusionAhrs->quaternion);

    // Calculate linear acceleration
    const FusionVector3 gravity = {
        .axis.x = 2.0f * (Q.x * Q.z - Q.w * Q.y),
        .axis.y = 2.0f * (Q.w * Q.x + Q.y * Q.z),
        .axis.z = 2.0f * (Q.w * Q.w - 0.5f + Q.z * Q.z),
    }; // equal to 3rd column of rotation matrix representation
    fusionAhrs->linearAcceleration = FusionVectorSubtract(accelerometer, gravity);

#undef Q // undefine shorthand label
}

/**
 * @brief Updates the AHRS algorithm.  This function should be called for each
 * new gyroscope measurement.
 * @param fusionAhrs AHRS algorithm structure.
 * @param gyroscope Gyroscope measurement in degrees per second.
 * @param accelerometer Accelerometer measurement in g.
 * @param samplePeriod Sample period in seconds.  This is the difference in time
 * between the current and previous gyroscope measurements.
 */
void FusionAhrsUpdateWithoutMagnetometer(FusionAhrs * const fusionAhrs, const FusionVector3 gyroscope, const FusionVector3 accelerometer, const float samplePeriod) {

    // Update AHRS algorithm
    FusionAhrsUpdate(fusionAhrs, gyroscope, accelerometer, FUSION_VECTOR3_ZERO, samplePeriod);

    // Zero yaw once initialisation complete
    if (FusionAhrsIsInitialising(fusionAhrs) == true) {
        fusionAhrs->zeroYawPending = true;
    } else {
        if (fusionAhrs->zeroYawPending == true) {
            FusionAhrsSetYaw(fusionAhrs, 0.0f);
            fusionAhrs->zeroYawPending = false;
        }
    }
}

/**
 * @brief Gets the quaternion describing the sensor relative to the Earth.
 * @param fusionAhrs AHRS algorithm structure.
 * @return Quaternion describing the sensor relative to the Earth.
 */
FusionQuaternion FusionAhrsGetQuaternion(const FusionAhrs * const fusionAhrs) {
    return FusionQuaternionConjugate(fusionAhrs->quaternion);
}

/**
 * @brief Gets the linear acceleration measurement equal to the accelerometer
 * measurement with the 1 g of gravity removed.
 * @param fusionAhrs AHRS algorithm structure.
 * @return Linear acceleration measurement.
 */
FusionVector3 FusionAhrsGetLinearAcceleration(const FusionAhrs * const fusionAhrs) {
    return fusionAhrs->linearAcceleration;
}

/**
 * @brief Gets the Earth acceleration measurement equal to linear acceleration
 * in the Earth coordinate frame.
 * @param fusionAhrs AHRS algorithm structure.
 * @return Earth acceleration measurement.
 */
FusionVector3 FusionAhrsGetEarthAcceleration(const FusionAhrs * const fusionAhrs) {
#define Q fusionAhrs->quaternion.element // define shorthand labels for more readable code
#define A fusionAhrs->linearAcceleration.axis
    const float qwqw = Q.w * Q.w; // calculate common terms to avoid repeated operations
    const float qwqx = Q.w * Q.x;
    const float qwqy = Q.w * Q.y;
    const float qwqz = Q.w * Q.z;
    const float qxqy = Q.x * Q.y;
    const float qxqz = Q.x * Q.z;
    const float qyqz = Q.y * Q.z;
    const FusionVector3 earthAcceleration = {
        .axis.x = 2.0f * ((qwqw - 0.5f + Q.x * Q.x) * A.x + (qxqy - qwqz) * A.y + (qxqz + qwqy) * A.z),
        .axis.y = 2.0f * ((qxqy + qwqz) * A.x + (qwqw - 0.5f + Q.y * Q.y) * A.y + (qyqz - qwqx) * A.z),
        .axis.z = 2.0f * ((qxqz - qwqy) * A.x + (qyqz + qwqx) * A.y + (qwqw - 0.5f + Q.z * Q.z) * A.z),
    }; // transpose of a rotation matrix representation of the quaternion multiplied with the linear acceleration
    return earthAcceleration;
#undef Q // undefine shorthand label
#undef A
}

/**
 * @brief Reinitialise the AHRS algorithm.
 * @param fusionAhrs AHRS algorithm structure.
 */
void FusionAhrsReinitialise(FusionAhrs * const fusionAhrs) {
    fusionAhrs->quaternion = FUSION_QUATERNION_IDENTITY;
    fusionAhrs->linearAcceleration = FUSION_VECTOR3_ZERO;
    fusionAhrs->rampedGain = INITIAL_GAIN;
}

/**
 * @brief Returns true while the AHRS algorithm is initialising.
 * @param fusionAhrs AHRS algorithm structure.
 * @return True while the AHRS algorithm is initialising.
 */
bool FusionAhrsIsInitialising(const FusionAhrs * const fusionAhrs) {
    return fusionAhrs->rampedGain > fusionAhrs->gain;
}

/**
 * @brief Sets the yaw component of the orientation measurement provided by the
 * AHRS algorithm.  This function can be used to reset drift in yaw when the
 * AHRS algorithm is being used without a magnetometer.
 * @param fusionAhrs AHRS algorithm structure.
 * @param yaw Yaw angle in degrees.
 */
void FusionAhrsSetYaw(FusionAhrs * const fusionAhrs, const float yaw) {
#define Q fusionAhrs->quaternion.element // define shorthand label for more readable code
    fusionAhrs->quaternion = FusionQuaternionNormalise(fusionAhrs->quaternion); // quaternion must be normalised accurately (approximation not sufficient)
    const float inverseYaw = atan2f(Q.x * Q.y + Q.w * Q.z, Q.w * Q.w - 0.5f + Q.x * Q.x); // Euler angle of conjugate
    const float halfInverseYawMinusOffset = 0.5f * (inverseYaw - FusionDegreesToRadians(yaw));
    const FusionQuaternion inverseYawQuaternion = {
        .element.w = cosf(halfInverseYawMinusOffset),
        .element.x = 0.0f,
        .element.y = 0.0f,
        .element.z = -1.0f * sinf(halfInverseYawMinusOffset),
    };
    fusionAhrs->quaternion = FusionQuaternionMultiply(inverseYawQuaternion, fusionAhrs->quaternion);
#undef Q // undefine shorthand label
}

//------------------------------------------------------------------------------
// End of file
