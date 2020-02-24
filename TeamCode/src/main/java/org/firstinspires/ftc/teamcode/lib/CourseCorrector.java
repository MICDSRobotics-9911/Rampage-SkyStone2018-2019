package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.lib.perceptron.AxisQuadrants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltageObj;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TranslateData;
import org.firstinspires.ftc.teamcode.robotplus.hardware.I2CGyroWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;

/**
 * @deprecated
 */
public class CourseCorrector {
    private TimeOffsetVoltageObj timeOffsetVoltageObj;
    private ElapsedTime startTime;
    private double endTime;
    private final int TOLERANCE = 5; // 5 degree tolerance

    public CourseCorrector(int desired, double voltage, ElapsedTime startTime) {
        // reset the counter
        this.timeOffsetVoltageObj = new TimeOffsetVoltageObj(desired, voltage);
        startTime.reset();
        this.startTime = startTime;

        // then set the target time
        this.endTime = startTime.milliseconds() + timeOffsetVoltageObj.getTimeExpected();
    }

    public boolean isFinished() {
        if (this.startTime.milliseconds() < endTime) {
            return false;
        }
        else {
            return true;
        }
    }

    // we want to use the I2C gyro for accuracy purposes
    public boolean isOnCourse(I2CGyroWrapper heading) {
        if (AxisQuadrants.roundAngle(heading.getHeading()) > TOLERANCE) {
            // this means the robot is off course
            return false;
        }
        else {
            return true;
        }
    }


    public void linearTranslate(TranslateData translation, I2CGyroWrapper gyro) {
        // first off, start the motors (we'll assume it's in the correct initial direction
        translation.getMecanumDrive().complexDrive(translation.getDirection().angle(), translation.getVelocity(), translation.getRotation());

        while (!this.isFinished()) {
            if (!this.isOnCourse(gyro)) {
                // it's not on course, so we need to correct it based on its current quadrant
                // from the relative quadrant, we know how far to the left we are, so turn the robot right
                translation.getMecanumDrive().stopMoving();
                while (AxisQuadrants.roundAngle(gyro.getHeading()) > TOLERANCE) {
                    translation.getMecanumDrive().complexDrive(MecanumDrive.Direction.UP.angle(), 0, 0.4);
                }
                // if we've gotten this far, the robot is once again on course, so continue
                translation.getMecanumDrive().complexDrive(translation.getDirection().angle(), translation.getVelocity(), translation.getRotation());
            }
        }
        translation.getMecanumDrive().stopMoving();
    }
}
