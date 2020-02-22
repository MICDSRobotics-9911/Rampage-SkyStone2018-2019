package org.firstinspires.ftc.teamcode.test;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.ClampState;
import org.firstinspires.ftc.teamcode.lib.GrabberState;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@TeleOp(name = "Second Robot DT", group = "Basic")
public class RampageSecondRobot extends OpMode {

    private Robot robot;
    private MecanumDrive mecanumDrive;


    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();

    }

    @Override
    public void loop() {

        this.mecanumDrive.complexDrive(gamepad1.right_stick_x, -gamepad1.left_stick_y, gamepad1.left_stick_x, telemetry);





    }
}
