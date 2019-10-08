package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@TeleOp(name = "9911DT", group = "Basic")
public class BasicDriveTrain extends OpMode {
    private Robot robot;
    private MecanumDrive mecanumDrive;
    private MotorPair intake;
    private DcMotor arm;
    private CRServo grabber;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.grabber = hardwareMap.get(CRServo.class, "grabber");
    }

    @Override
    public void loop() {
        this.mecanumDrive.complexDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        // grabber
        if (gamepad1.right_bumper) {
            this.grabber.setPower(0.5);
        }
        else if (gamepad1.left_bumper) {
            this.grabber.setPower(-0.5);
        }
        else {
            this.grabber.setPower(0);
        }

        // intake
        if (gamepad1.dpad_left) {
            this.intake.getMotor1().setPower(1);
            this.intake.getMotor2().setPower(-1);
        }
        else if (gamepad1.dpad_right) {
            this.intake.getMotor1().setPower(-1);
            this.intake.getMotor2().setPower(1);
        }

        if (gamepad1.dpad_up) {
            this.arm.setPower(1);
        }
        else if (gamepad1.dpad_down) {
            this.arm.setPower(-1);
        }
        else {
            this.arm.setPower(0);
        }

        // stop
        if (gamepad1.x) {
            this.intake.stopMoving();
            this.mecanumDrive.stopMoving();
            this.grabber.setPower(0);
            this.arm.setPower(0);
        }
    }
}
