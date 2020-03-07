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
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@Autonomous(name = "BlueFromSquareToLine", group = "Generic")
public class BlueFromSquareToLine extends LinearOpMode implements AutonomousConstants {

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

                case 1:
                this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(),1,0);
                sleep(10);
                this.mecanumDrive.stopMoving();
                step++;
                break;


                case 2:
                this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(),1,0);
                sleep(500);
                this.mecanumDrive.stopMoving();
                step++;
                break;

            }
        }
    }
}
