package org.firstinspires.ftc.teamcode.autonomii.experimental;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.autonomii.RedMoveFoundationTriangle;
import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * @deprecated
 * @see RedMoveFoundationTriangle
 */
@Autonomous(name = "RedMoveFoundation", group = "Generic")
@Disabled
public class RedMoveFoundation extends LinearOpMode implements AutonomousConstants {

    private Robot robot;
    private MecanumDrive mecanumDrive;
    private ColorSensor colorSensor;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;
    private Servo assist;
    private Servo clampLeft;
    private Servo clampRight;
    private TouchSensor touchSensorLeft;
    private TouchSensor touchSensorRight;
    private DigitalChannel frontSwitch;
    private double voltage;


    private float hsvValues[] = {0F, 0F, 0F};
    private final double SCALE_FACTOR = 355;
    private int step = 1;

    public void runOpMode() {
        // init
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.colorSensor = hardwareMap.get(ColorSensor.class, "c1");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");
        this.assist = hardwareMap.get(Servo.class, "assist");
        this.clampLeft = hardwareMap.get(Servo.class, "clamp_left");
        this.clampRight = hardwareMap.get(Servo.class, "clamp_right");
        this.touchSensorLeft = hardwareMap.get(TouchSensor.class, "left_touch");
        this.touchSensorRight = hardwareMap.get(TouchSensor.class, "right_touch");
        this.frontSwitch = hardwareMap.get(DigitalChannel.class, "front_switch");
        this.voltage = hardwareMap.voltageSensor.get("Expansion Hub 10").getVoltage();

        // set motors to coast
        this.mecanumDrive.getMajorDiagonal().getMotor1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.mecanumDrive.getMajorDiagonal().getMotor2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.mecanumDrive.getMinorDiagonal().getMotor1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.mecanumDrive.getMinorDiagonal().getMotor2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

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
                    // move backwards
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(500);
                    this.mecanumDrive.stopMoving();
                    this.step++;
                    break;
                case 1:
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), -1, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(voltage, 160));
                    this.mecanumDrive.stopMoving();
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), 1, 0);
                    sleep(1100);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 2:
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 0.5, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 100));
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 0.45, 0);
                    sleep(550);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 3:
                    // clamp the foundation
                    this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_DOWN);
                    this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_DOWN);
                    sleep(1500);
                    step++;
                    break;
                case 4:
                    // move the foundation until the distance to wall is met
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0.1);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 135));
                    this.mecanumDrive.stopMoving();
                    // bump off the wall
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(190);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 5:
                    // take clamp off and move to the red line
                    this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_UP);
                    this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_UP);
                    sleep(1400);
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.RIGHT.angle(), -1, -0.015);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 260));
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
            }
        }
    }
}
