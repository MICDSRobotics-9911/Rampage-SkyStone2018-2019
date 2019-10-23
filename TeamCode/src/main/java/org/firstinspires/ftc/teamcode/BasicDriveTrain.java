package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    private CRServo grabber;

    // gamepad states

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(CRServo.class, "grabber");
    }

    @Override
    public void loop() {
        this.mecanumDrive.complexDrive(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        // grabber
        this.grabber.setPower((gamepad1.dpad_left) ? 0.5 : 0);
        this.grabber.setPower((gamepad1.dpad_right) ? -0.5 : 0);

        // arm
        this.arm.setPower((gamepad1.dpad_up) ? -1 : 0);
        this.arm.setPower((gamepad1.dpad_up) ? 1 : 0);
        this.arm.setPower((gamepad1.dpad_down) ? 1 : 0);
        this.arm.setPower((gamepad1.dpad_down) ? -1 : 0);

        // intake
        this.intake.getMotor1().setPower((gamepad1.x) ? 1 : 0);
        this.intake.getMotor2().setPower((gamepad1.x) ? -1 : 0);
        this.intake.getMotor1().setPower((gamepad1.y) ? -1 : 0);
        this.intake.getMotor2().setPower((gamepad1.y) ? 1 : 0);

        // elevator
        this.elevator.setPower((gamepad1.a) ? 1 : 0);
        this.elevator.setPower((gamepad1.b) ? -1 : 0);

        // stop
        if (gamepad1.right_bumper) {
            this.intake.stopMoving();
            this.mecanumDrive.stopMoving();
            this.grabber.setPower(0);
            this.arm.setPower(0);
            this.elevator.setPower(0);
        }
    }
}
