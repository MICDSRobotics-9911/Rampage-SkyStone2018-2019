package org.firstinspires.ftc.teamcode.autonomii;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

import java.util.concurrent.TimeoutException;

@Autonomous(name = "TESTBlueFull", group = "Generic")
public class BlueFull extends LinearOpMode implements AutonomousConstants, TeleOpConstants {

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
    private MotorPair intake;

    private float hsvValues[] = {0F, 0F, 0F};
    private final double SCALE_FACTOR = 355;
    private int step = -5;

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
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");

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
                // TODO: may have to implement code for purging our capstone (orange block)

                // first we have to approach the stones
                case -5:

                    this.intake.getMotor1().setPower(1);
                    this.intake.getMotor2().setPower(-1);

                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(voltage, 53)); // TODO: adjust first distance to the skystones
                    this.mecanumDrive.stopMoving();
                    // we should now be in position to start scanning the bricks
                    step++;
                    break;
                case -4:
                    // start going down the line, scanning for skystones
                    this.intake.getMotor1().setPower(0);
                    this.intake.getMotor2().setPower(0);
                    if ((((int) this.hsvValues[0]) < 60)) {
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1, 0);
                    } else {
                        // found a skystone (hopefully, anyway)
                        this.mecanumDrive.stopMoving();
                        step++;
                    }
                    break;
                case -3:
                    // move backwards, so we clear the main skybridge
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
                    // TODO: 11/18/2019 implement voltage sensor
                    sleep(TimeOffsetVoltage.calculateDistance(voltage, 8)); // TODO: make sure this value puts the robot away from the skybridge
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case -2:
                    // drop the arm to grab a skystone
                    this.arm.setPower(-0.5);
                    sleep(AutonomousConstants.ARM_DROP_DISTANCE); // if you want to change this, make sure you change it in AutonomousConstants
                    this.arm.setPower(0);
                    step++;
                    break;
                case -1:
                    // open the claw and grab the skystone
                    this.grabber.setPosition(TeleOpConstants.GRABBER_PARTIAL_OPEN);
                    this.assist.setPosition(TeleOpConstants.ASSIST_CLOSED);
                    sleep(1500);
                    this.grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
                    step++;
                    break;
                case 0:
                    // move arm back up a bit
                    this.arm.setPower(1);
                    sleep(AutonomousConstants.ARM_DROP_DISTANCE);
                    this.arm.setPower(0);
                    step++;
                    break;
                case 1:
                    // start translating to the other side of the field
                    // the next step will be to start putting the foundation in the right spot
                    if (!this.touchSensorRight.isPressed()) {
                        // keep moving
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), 1, 0);
                    }
                    else {
                        this.mecanumDrive.stopMoving();
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), -1, 0);
                        sleep(250);
                        this.mecanumDrive.stopMoving();
                        step++;
                    }
                    break;
                case 2:
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 0.7, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 50));
                    //this.mecanumDrive.stopMoving();
                    // then slow down
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 0.45, 0);
                    sleep(1000);
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
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, -0.15);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 120));
                    this.mecanumDrive.stopMoving();
                    // bump off the wall
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(115);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 5:
                    // take clamp off and move to the blue line
                    this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_UP);
                    this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_UP);
                    sleep(1300);
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 225));
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
            }
        }
    }
}
