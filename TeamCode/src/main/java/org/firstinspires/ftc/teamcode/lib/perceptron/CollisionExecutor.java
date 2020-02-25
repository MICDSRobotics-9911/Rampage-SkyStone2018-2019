package org.firstinspires.ftc.teamcode.lib.perceptron;

import com.qualcomm.robotcore.hardware.IntegratingGyroscope;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;

/**
 * ML-based algorithm for detecting a collision
 */
public class CollisionExecutor {
    /**
     * Determines if the robot has collided with something. See the notebook for further details.
     * @param heading the current robot's heading
     * @param wrapper used to get the linear acceleration of the robot
     * @return whether the robot has collided
     */
    public static boolean calculate(int heading, IMUWrapper wrapper) {
        if (2 * ((AxisQuadrants.roundAngle(heading) * 0.35390572195504255) + (linearAccelerationMagnitude(wrapper.getIMU().getLinearAcceleration()) * 0.020031598717753236)) > 1) {
            return true;
        }
        else {
            return false;
        }
    }

    /**
     * Determines the magnitude of the linear acceleration in the x- and y- axes
     * @param current the acceleration object of the robot
     * @see Acceleration
     * @return the magnitude
     */
    private static float linearAccelerationMagnitude(Acceleration current) {
        //Acceleration current = this.imuWrapper.getIMU().getLinearAcceleration();
        return (float) Math.sqrt(Math.pow(current.xAccel,2) + Math.pow(current.yAccel,2));
    }

    /**
     * Determines the magnitude of the change in degrees in both axes
     * @deprecated
     * @param gyro the I2C gyro object
     * @return the magnitude
     */
    public static float deltaAngleMagnititude(IntegratingGyroscope gyro) {
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        return (float)Math.sqrt(Math.pow(rates.xRotationRate - 0.5,2) + Math.pow(rates.yRotationRate - 0.5,2));
    }
}
