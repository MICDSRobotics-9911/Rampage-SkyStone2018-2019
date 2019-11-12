package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@TeleOp(name = "9911DT", group = "Basic")
public class BasicDriveTrain extends OpMode {
    private Robot robot;
    private MecanumDrive mecanumDrive;
    private MotorPair intake;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;

    // gamepad states

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");

        this.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop() {
        this.mecanumDrive.complexDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        // grabber
        /*this.grabber.setPower((this.grabber.getPower() == 0) && (gamepad1.dpad_left || gamepad2.dpad_left) ? 1 : 0);
        this.grabber.setPower((this.grabber.getPower() == 0) && (gamepad1.dpad_right || gamepad2.dpad_right) ? -1 : 0);*/
        //this.grabber.setPosition((gamepad1.dpad_right || gamepad2.dpad_right) ? -1 : 0);
        if (gamepad1.right_bumper) {
            this.grabber.setPosition(1);
        }
        else {
            this.grabber.setPosition(0);
        }

        // arm
        this.arm.setPower((this.arm.getPower() == 0) && (gamepad1.dpad_up || gamepad2.dpad_up) ? 1 : 0);
        this.arm.setPower((this.arm.getPower() == 0) && (gamepad1.dpad_down || gamepad2.dpad_down) ? -1 : 0);
        /*this.arm.setPower((gamepad1.dpad_down || gamepad2.dpad_down) ? 1 : 0);
        this.arm.setPower((gamepad1.dpad_down || gamepad2.dpad_down) ? -1 : 0);*/

        // intake
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.x || gamepad2.x) ? 1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.x || gamepad2.x) ? -1 : 0);
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.y || gamepad2.y) ? -1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.y || gamepad2.y) ? 1 : 0);

        // elevator
        this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.dpad_right) ? 1 : 0);
        this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.left_bumper) ? -1 : 0);

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
