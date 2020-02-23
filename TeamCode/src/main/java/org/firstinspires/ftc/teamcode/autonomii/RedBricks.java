package org.firstinspires.ftc.teamcode.autonomii;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@Autonomous(name = "RedBricks", group = "Red")
public class RedBricks extends LinearOpMode implements AutonomousConstants, TeleOpConstants {

    private Robot robot;
    private MecanumDrive mecanumDrive;
    private ColorSensor colorSensor;
    private ColorSensor lucasDetector;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;
    private Servo assist;
    private Servo clampLeft;
    private Servo clampRight;
    private TouchSensor touchSensorLeft;
    private TouchSensor touchSensorRight;
    private DigitalChannel frontSwitch;
    private IMUWrapper imuWrapper;
    private double voltage;
    private MotorPair intake;


    private float hsvValues[] = {0F, 0F, 0F};
    private float lucasValues[] = {0F, 0F, 0F};
    private final double SCALE_FACTOR = 355;
    private int step = -5;

    public void runOpMode() {
        // init
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.colorSensor = hardwareMap.get(ColorSensor.class, "c1");
        this.lucasDetector = hardwareMap.get(ColorSensor.class, "lucasDetector");
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
        this.imuWrapper = new IMUWrapper(hardwareMap);


        // brakes!
        this.mecanumDrive.getMinorDiagonal().getMotor1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mecanumDrive.getMinorDiagonal().getMotor2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mecanumDrive.getMajorDiagonal().getMotor1().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.mecanumDrive.getMajorDiagonal().getMotor2().setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            Color.RGBToHSV((int) (lucasDetector.red() * this.SCALE_FACTOR),
                    (int) (lucasDetector.green() * this.SCALE_FACTOR),
                    (int) (lucasDetector.blue() * this.SCALE_FACTOR),
                    this.lucasValues
            );


            telemetry.addData("Step", this.step);
            telemetry.addData("Red", this.lucasValues[0]);
            telemetry.addData("Green", this.lucasValues[1]);
            telemetry.addData("Blue", this.lucasValues[2]);
            telemetry.addData("Alpha", this.lucasDetector.alpha());
            telemetry.update();

            switch (step) {
                // TODO: may have to implement code for purging our capstone (orange block)

                // first we have to approach the stones
                case -5:

                    /*this.intake.getMotor1().setPower(1);
                    this.intake.getMotor2().setPower(-1);*/

                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(TimeOffsetVoltage.calculateDistance(voltage, 52)); // TODO: adjust first distance to the skystones
                    this.mecanumDrive.stopMoving();
                    // we should now be in position to start scanning the bricks
                    step++;
                    break;
                case -4:
                    // start going down the line, scanning for skystones

                    if ((((int) this.hsvValues[0]) < 85)) {
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.RIGHT.angle(), -0.9, 0.05);
                    } else {
                        // found a skystone (hopefully, anyway)
                        this.mecanumDrive.stopMoving();
                        step++;
                    }
                    break;
                case -3:
                    /*
                    // move backwards, so we clear the main skybridge
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0);
                    sleep(150);
                    this.mecanumDrive.stopMoving();
                    */
                    step++;
                    break;
                case -2:
                    // drop the arm to grab a skystone
                    this.arm.setPower(-0.5);
                    sleep(AutonomousConstants.ARM_DROP_DISTANCE); // if you want to change this, make sure you change it in AutonomousConstants
                    this.arm.setPower(0);
                    sleep(300);
                    step++;
                    break;
                case -1:
                    // open the claw and grab the skystone
                    this.grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
                    this.assist.setPosition(0.1); // 'u' is the assist
                    sleep(150);
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(200);
                    this.mecanumDrive.stopMoving();
                    this.assist.setPosition(1);
                    sleep(1300);


                    step++;
                    break;
                case 0:
                    // move arm back up a bit
                    this.arm.setPower(1);
                    sleep(AutonomousConstants.ARM_DROP_DISTANCE / 7);
                    this.arm.setPower(0.01);

                    // implement double check

                    if ((((int) lucasDetector.alpha()) < 200)) {

                        this.assist.setPosition(0.1); // 'u' is the assist
                        this.arm.setPower(0.3);
                        sleep(AutonomousConstants.ARM_DROP_DISTANCE / 16); // if you want to change this, make sure you change it in AutonomousConstants
                        this.arm.setPower(0);
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0);
                        this.sleep(300);
                        this.mecanumDrive.stopMoving();
                        this.arm.setPower(-0.5);
                        sleep(AutonomousConstants.ARM_DROP_DISTANCE / 6); // if you want to change this, make sure you change it in AutonomousConstants
                        this.arm.setPower(0);
                        sleep(300);
                        this.grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
                        this.assist.setPosition(0.1); // 'u' is the assist
                        sleep(150);
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                        sleep(200);
                        this.mecanumDrive.stopMoving();
                        this.assist.setPosition(1);
                        sleep(1300);


                    }

                    // move backwards
                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0);
                    sleep(450);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 1:
                    // start translating to the other side of the field
                    // the next step will be to start putting the foundation in the right spot
                    this.imuWrapper.updateAngles();
                    sleep(1); // just so we don't burn a hole in the CPU :)
                    float angle = this.imuWrapper.getHeading();
                    if (angle <= 73) { // !this.touchSensorRight.isPressed()
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), 0, 0.4); // TODO: may need to change the sign
                    } else {
                        // start moving towards the wall and hit the wall
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0);
                        sleep(TimeOffsetVoltage.calculateDistance(voltage, 165));
                        this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                        sleep(TimeOffsetVoltage.calculateDistance(voltage, 25));
                        this.mecanumDrive.stopMoving();

                    }
            }
        }
    }
}
