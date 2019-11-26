package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.GrabberState;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@TeleOp(name = "9911DT", group = "Basic")
public class BasicDriveTrain extends OpMode implements TeleOpConstants {
    private Robot robot;
    private MecanumDrive mecanumDrive;
    private MotorPair intake;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;
    private Servo assist;
    private Servo clamp;
    //private boolean isGrabberLocked = true;
    private GrabberState grabberState = GrabberState.CLOSED;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");
        this.assist = hardwareMap.get(Servo.class, "assist");

        this.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        telemetry.addData("isGrabberOpen", this.grabberState);
        telemetry.update();

        this.mecanumDrive.complexDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        // assist
        this.assist.setPosition((gamepad1.right_bumper || gamepad2.right_bumper) ? 0 : 0.65); // !this.isGrabberLocked

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

        // TODO: 11/25/2019 need to break this ternary out
        // this.grabber.setPosition(((gamepad1.right_bumper || gamepad2.right_bumper)) ? 0.25 : 0.15);

        // arm
        this.arm.setPower(gamepad2.left_stick_y * -0.5);

        // intake
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.x || gamepad2.x) ? 1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.x || gamepad2.x) ? -1 : 0);
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.y || gamepad2.y) ? -1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.y || gamepad2.y) ? 1 : 0);

        // elevator
        this.elevator.setPower(-gamepad2.right_stick_y);
        /*this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.dpad_right) ? 1 : 0);
        this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.left_bumper) ? -1 : 0);*/

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
