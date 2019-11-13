package org.firstinspires.ftc.teamcode.autonomii;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@Autonomous(name = "DetectSkyStone", group = "Test")
public class DetectSkyStone extends LinearOpMode {

    private Robot robot;
    private MecanumDrive mecanumDrive;
    private ColorSensor colorSensor;


    private float hsvValues[] = {0F, 0F, 0F};
    private final double SCALE_FACTOR = 355;
    private int step = 0;

    public void runOpMode() {
        // init
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.colorSensor = hardwareMap.get(ColorSensor.class, "c1");

        waitForStart();


        while (opModeIsActive()) {
            // update colors
            Color.RGBToHSV((int)(colorSensor.red() * this.SCALE_FACTOR),
                    (int)(colorSensor.green() * this.SCALE_FACTOR),
                    (int)(colorSensor.blue() * this.SCALE_FACTOR),
                    this.hsvValues
            );

            telemetry.addData("Step", this.step);
            telemetry.addData("Red", this.hsvValues[0]);
            telemetry.addData("Green", this.hsvValues[1]);
            telemetry.addData("Blue", this.hsvValues[2]);
            telemetry.addData("Alpha", this.colorSensor.alpha());
            telemetry.update();

            if ((((int)this.hsvValues[0]) < 85)) {
                this.mecanumDrive.complexDrive(0, 1, 0);
            }
            else {
                // found a skystone
                //this.step++;
                this.mecanumDrive.stopMoving();
            }
        }
    }
}
