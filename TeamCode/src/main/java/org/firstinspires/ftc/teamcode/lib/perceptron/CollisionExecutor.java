package org.firstinspires.ftc.teamcode.lib.perceptron;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.lib.anglecorrection.AxisQuadrants;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;

public class CollisionExecutor {
    public static boolean calculate(int heading, IMUWrapper wrapper) {
        if (2 * ((roundAngle(heading) * 0.35390572195504255) + (linearAccelerationMagnitude(wrapper.getIMU().getLinearAcceleration()) * 0.020031598717753236)) > 1) {
            return false;
        }
        else {
            return true;
        }
    }

    private static float roundAngle(float angle) {
        float correctedAngle = angle;
        AxisQuadrants quadrants = AxisQuadrants.Q1;
        if (correctedAngle > 180) {
            correctedAngle = 180 - angle;
        }

        if (correctedAngle > 45 && correctedAngle < 90) {
            quadrants = AxisQuadrants.Q2;
        }
        else if (correctedAngle > 90 && correctedAngle < 135) {
            quadrants = AxisQuadrants.Q3;
        }
        else if (correctedAngle > 135) {
            quadrants = AxisQuadrants.Q4;
        }
        else {
            quadrants = AxisQuadrants.Q1;
        }

        float deltaAngle = 0;
        switch (quadrants) {
            case Q1: deltaAngle = 45 - correctedAngle; break;
            case Q2: deltaAngle = 90 - correctedAngle; break;
            case Q3: deltaAngle = 135 - correctedAngle; break;
            case Q4: deltaAngle = 180 - correctedAngle; break;
        }

        return deltaAngle;
    }

    private static float linearAccelerationMagnitude(Acceleration current) {
        //Acceleration current = this.imuWrapper.getIMU().getLinearAcceleration();
        return (float) Math.sqrt(Math.pow(current.xAccel,2) + Math.pow(current.yAccel,2));
    }

    public static float deltaAngleMagnititude(IntegratingGyroscope gyro) {
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        return (float)Math.sqrt(Math.pow(rates.xRotationRate - 0.5,2) + Math.pow(rates.yRotationRate - 0.5,2));
    }
}
