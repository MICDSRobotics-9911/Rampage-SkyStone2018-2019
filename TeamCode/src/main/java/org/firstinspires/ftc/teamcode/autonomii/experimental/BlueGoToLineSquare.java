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

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

/**
 * @deprecated
 */
@Autonomous(name = "RedGoToLine", group = "Generic")
@Disabled
public class BlueGoToLineSquare extends LinearOpMode implements AutonomousConstants {

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
                case 0:
                    // take clamp off and move to the blue line

                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
                    sleep(100);
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
                case 1:
                    this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_UP);
                    this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_UP);

                    this.mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), 1, 0.002);
                    sleep(TimeOffsetVoltage.calculateDistance(this.voltage, 225));
                    this.mecanumDrive.stopMoving();
                    step++;
                    break;
            }
        }
    }
}
