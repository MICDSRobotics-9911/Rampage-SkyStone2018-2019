package org.firstinspires.ftc.teamcode.lib.perceptron;

import org.firstinspires.ftc.robotcore.external.navigation.Axis;

/**
 * Enum for notating what relative angle quadrant the robot is in. See notebook for further details.
 */
public enum AxisQuadrants {
    Q1, Q2, Q3, Q4;

    /**
     * Measures the heading of the robot against the closest 45 degrees
     * @param angle
     * @see AxisQuadrants
     * @return
     */
    public static float roundAngle(float angle) {
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

    public AxisQuadrants currentQuadrant(float angle) {
        float correctedAngle = angle;
        if (correctedAngle > 180) {
            correctedAngle = 180 - angle;
        }

        if (correctedAngle > 45 && correctedAngle < 90) {
            return AxisQuadrants.Q2;
        }
        else if (correctedAngle > 90 && correctedAngle < 135) {
            return AxisQuadrants.Q3;
        }
        else if (correctedAngle > 135) {
            return AxisQuadrants.Q4;
        }
        else {
            return AxisQuadrants.Q1;
        }
    }
}