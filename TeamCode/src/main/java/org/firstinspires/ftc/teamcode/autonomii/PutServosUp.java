package org.firstinspires.ftc.teamcode.autonomii;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

import java.util.concurrent.TimeoutException;

/**
 * @deprecated
 */
@Autonomous(name = "PutServosUp", group = "Blue")
@Disabled
public class PutServosUp extends LinearOpMode implements AutonomousConstants, TeleOpConstants {
    //this is a test
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

                case -5:

                    //puts both clamps up so that we can install them correctly
                    this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_UP);
                    this.clampRight.setPosition((AutonomousConstants.CLAMP_RIGHT_UP));
                    

                    step++;
                    break;

            }
        }
    }
}
