package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.ClampState;
import org.firstinspires.ftc.teamcode.lib.GrabberState;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.lib.perceptron.CollisionExecutor;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;
import java.lang.Math;

@TeleOp(name = "9911DT", group = "Basic")
public class logDT extends OpMode implements TeleOpConstants, AutonomousConstants {
    private Robot robot;
    private MecanumDrive mecanumDrive;
    private MotorPair intake;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;
    private Servo assist;
    private Servo clampLeft;
    private Servo clampRight;
    private GrabberState grabberState = GrabberState.CLOSED;
    private ClampState clampState = ClampState.UP;
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private AngularVelocity rates;
    private IMUWrapper imuWrapper;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");
        this.assist = hardwareMap.get(Servo.class, "assist");
        this.clampLeft = hardwareMap.get(Servo.class, "clamp_left");
        this.clampRight = hardwareMap.get(Servo.class, "clamp_right");
        this.modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        this.imuWrapper = new IMUWrapper(hardwareMap);
        /*this.gyro = (IntegratingGyroscope) modernRoboticsI2cGyro;
        modernRoboticsI2cGyro.calibrate();*/

        this.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        //rates = gyro.getAngularVelocity(AngleUnit.DEGREES.DEGREES);
        telemetry.addData("isGrabberOpen", this.grabberState);
        //telemetry.addData("Collision Detected", CollisionExecutor.calculate(modernRoboticsI2cGyro.getHeading(), this.imuWrapper));
        telemetry.update();

        // check for collision. If collided, stop to prevent further damage
        /*if (CollisionExecutor.calculate(modernRoboticsI2cGyro.getHeading(), this.imuWrapper)) {
            this.mecanumDrive.stopMoving();
        }*/

        this.mecanumDrive.complexDrive(-(Math.pow(10, (Math.abs(gamepad1.left_stick_x))-1.02288)) + 0.052, (Math.pow(10, (Math.abs(gamepad1.left_stick_y))-1.02288)) + 0.052, (Math.pow(10, (Math.abs(gamepad1.right_stick_x))-1.02288)) + 0.052, telemetry);

        // assist
        this.assist.setPosition((gamepad1.right_bumper || gamepad2.right_bumper) ? TeleOpConstants.ASSIST_CLOSED : TeleOpConstants.ASSIST_OPEN);

        // grabber
        if (this.grabberState.equals(GrabberState.CLOSED) && (gamepad2.right_bumper || gamepad1.right_bumper)) {
            // we want to open it partially then
            this.grabberState = GrabberState.PARTIAL;
        }
        else if (this.grabberState.equals(GrabberState.PARTIAL) && !(gamepad2.right_bumper || gamepad1.right_bumper)) {
            // we want to close it by default then then
            this.grabberState = GrabberState.CLOSED;
        }
        else if (gamepad2.dpad_left || gamepad1.dpad_left) {
            // fully open the grabber
            this.grabberState = GrabberState.OPEN;
        }
        else if (this.grabberState.equals(GrabberState.OPEN) && (gamepad2.dpad_right || gamepad1.dpad_right)) {
            // close the grabber from the full position
            this.grabberState = GrabberState.CLOSED;
        }

        // grabber servo enum controller switch
        switch (this.grabberState) {
            case OPEN:
                this.grabber.setPosition(TeleOpConstants.GRABBER_FULL_OPEN);
                break;
            case PARTIAL:
                this.grabber.setPosition(TeleOpConstants.GRABBER_PARTIAL_OPEN);
                break;
            case CLOSED:
                this.grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
                break;
        }

        // arm
        this.arm.setPower(gamepad2.left_stick_y * 0.5);

        // intake
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.x || gamepad2.x) ? 1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.x || gamepad2.x) ? -1 : 0);
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.y || gamepad2.y) ? -1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.y || gamepad2.y) ? 1 : 0);

        // elevator
        this.elevator.setPower(gamepad2.right_stick_y);
        /*this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.dpad_right) ? 1 : 0);
        this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.left_bumper) ? -1 : 0);*/

        // clamp
        if (this.clampState.equals(ClampState.UP) && (gamepad2.dpad_down || gamepad1.dpad_down)) {
            // close the clamp
            this.clampState = ClampState.DOWN;
        }
        else if (this.clampState.equals(ClampState.DOWN) && (gamepad2.dpad_up || gamepad1.dpad_up)) {
            // raise the clamp
            this.clampState = ClampState.UP;
        }

        // clamp servo switch controller
        switch (this.clampState) {
            case UP:
                this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_UP);
                this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_UP);
                break;
            case DOWN:
                this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_DOWN);
                this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_DOWN);
                break;
        }

        // stop
        if (gamepad1.right_bumper) {
            this.intake.stopMoving();
            this.mecanumDrive.stopMoving();
            //this.grabber.setPower(0);
            this.arm.setPower(0);
            this.elevator.setPower(0);
        }
    }
}
