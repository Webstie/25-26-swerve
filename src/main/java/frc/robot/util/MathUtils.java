// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Optional;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import lombok.experimental.UtilityClass;

@UtilityClass
public class MathUtils {
    public static final <T extends Number> T clamp(T value, T min, T max) {
        if (value.doubleValue() < min.doubleValue())
            return min;
        else if (value.doubleValue() > max.doubleValue())
            return max;
        else
            return value;
    }

    public static final double get2dVelocity(ChassisSpeeds chassisSpeeds) {
        return Math.sqrt(chassisSpeeds.vxMetersPerSecond * chassisSpeeds.vxMetersPerSecond
                + chassisSpeeds.vyMetersPerSecond * chassisSpeeds.vyMetersPerSecond);
    }

    public static final <T extends Number> boolean inRange(T value, T minInc, T maxInc) {
        return value.doubleValue() >= minInc.doubleValue() && value.doubleValue() <= maxInc.doubleValue();
    }

    public static final double roundToPlace(double x, int place) {
        var pow10 = Math.pow(10, place);
        return Math.round(x * pow10) / pow10;
    }

    // doesn't belong here but whatever
    public static final <T> T orIfNull(T value, T _default) {
        return value == null ? _default : value;
    }

    public static final <T> Optional<T> arrayGetSafe(T[] arr, int idx) {
        return idx >= arr.length ? Optional.empty() : Optional.of(arr[idx]);
    }

    public static final <T> T printAndReturn(T value, String prefix, String suffix) {
        System.out.println(prefix + value + suffix);
        return value;
    }

    public static final int boolToInt(boolean b) {
        return b ? 1 : 0;
    }

    public static final double[] rotateVector(double[] vec, double thetaRad) {
        return new double[] {
                vec[0] * Math.cos(thetaRad) - vec[1] * Math.sin(thetaRad),
                vec[0] * Math.sin(thetaRad) + vec[1] * Math.cos(thetaRad),
        };
    }
}
