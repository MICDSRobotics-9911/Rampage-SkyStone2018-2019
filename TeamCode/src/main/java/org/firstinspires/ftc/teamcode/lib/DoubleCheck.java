package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;

public class DoubleCheck {
    public static void check(LinearOpMode lop, ColorSensor lucasDetector, DcMotor arm, MecanumDrive mecanumDrive, Servo grabber, Servo assist) {
        // implement double check
        if ((((int) lucasDetector.alpha()) < 200   )) {
            assist.setPosition(0.1); // 'u' is the assist
            arm.setPower(0.3);
            lop.sleep(AutonomousConstants.ARM_DROP_DISTANCE/16); // if you want to change this, make sure you change it in AutonomousConstants
            arm.setPower(0);
            mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0);
            lop.sleep(300);
            mecanumDrive.stopMoving();
            arm.setPower(-0.5);
            lop.sleep(AutonomousConstants.ARM_DROP_DISTANCE/6); // if you want to change this, make sure you change it in AutonomousConstants
            arm.setPower(0);
            lop.sleep(300);
            grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
            assist.setPosition(0.1); // 'u' is the assist
            lop.sleep(150);
            mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
            lop.sleep(200);
            mecanumDrive.stopMoving();
            assist.setPosition(1);
            lop.sleep(1300);
        }
    }
}