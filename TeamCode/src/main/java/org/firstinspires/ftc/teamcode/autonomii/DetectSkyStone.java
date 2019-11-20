package org.firstinspires.ftc.teamcode.autonomii;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@Autonomous(name = "DetectSkyStone", group = "Test")
public class DetectSkyStone extends LinearOpMode implements AutonomousConstants {

    private Robot robot;
    private MecanumDrive mecanumDrive;
    private ColorSensor colorSensor;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;
    private Servo assist;
    private Servo clamp;
    private TouchSensor touchSensorLeft;
    private TouchSensor touchSensorRight;


    private float hsvValues[] = {0F, 0F, 0F};
    private final double SCALE_FACTOR = 355;
    private int step = 0;

    public void runOpMode() {
        // init
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.colorSensor = hardwareMap.get(ColorSensor.class, "c1");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");
        this.assist = hardwareMap.get(Servo.class, "assist");
        this.clamp = hardwareMap.get(Servo.class, "clamp");
        this.touchSensorLeft = hardwareMap.get(TouchSensor.class, "left_touch");
        this.touchSensorRight = hardwareMap.get(TouchSensor.class, "right_touch");

        waitForStart();


        while (opModeIsActive()) {
            // update colors
            Color.RGBToHSV((int) (colorSensor.red() * this.SCALE_FACTOR),
                    (int) (colorSensor.green() * this.SCALE_FACTOR),
                    (int) (colorSensor.blue() * this.SCALE_FACTOR),
                    this.hsvValues
            );


            telemetry.addData("Step", this.step);
            telemetry.addData("Red", this.hsvValues[0]);
            telemetry.addData("Green", this.hsvValues[1]);
            telemetry.addData("Blue", this.hsvValues[2]);
            telemetry.addData("Alpha", this.colorSensor.alpha());
            telemetry.update();

            switch (step) {
                case 0:
                    this.clamp.setPosition(AutonomousConstants.CLAMP_UP);
                    // TODO: 11/18/2019 check to see if we've hit the depot wall using another color sensor
                    if ((((int) this.hsvValues[0]) < 85)) {
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1, 0);
                    } else {
                        // found a skystone
                        //this.step++;
                        this.mecanumDrive.stopMoving();
                        step++;
                    }
                    break;
                case 1:
                    // move backwards
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
                    // TODO: 11/18/2019 implement voltage sensor
                    sleep(150);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 2:
                    // drop the arm
                    this.arm.setPower(-0.5);
                    sleep(AutonomousConstants.ARM_DROP_DISTANCE);
                    this.arm.setPower(0);
                    step++;
                    break;
                case 3:
                    // open the claw and grab the skystone
                    this.grabber.setPosition(0);
                    sleep(1500);
                    //this.grabber.setPosition(0.5);
                    step++;
                    break;
                case 4:
                    // move arm back up a bit
                    this.arm.setPower(1);
                    sleep(AutonomousConstants.ARM_DROP_DISTANCE);
                    this.arm.setPower(0);
                    step++;
                    break;
                case 5:
                    // move towards the foundation by hitting a wall
                    telemetry.addData("button", this.touchSensorRight.getValue());
                    if (this.touchSensorRight.getValue() == 1) {
                        this.mecanumDrive.stopMoving();
                        this.step++;
                    }
                    else {
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), 1, 0);
                    }
                    break;
                case 6:
                    // move forwards
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(14.4, 55));
                    this.mecanumDrive.stopMoving();
                    // down the clamp
                    this.clamp.setPosition(0);
                    sleep(500);
                    this.step++;
                    break;
                case 7:
                    // drag the foundation until a button press (it's against a wall)
                    if (!this.touchSensorRight.isPressed()) {
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
                    }
                    else {
                        this.mecanumDrive.stopMoving();
                        this.step++;
                    }
                    break;
                case 8:
                    // open the clamp
                    this.clamp.setPosition(0.5);
                    this.step++;
                    break;
            }
        }
    }
}
