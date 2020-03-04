package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

public class DoubleCheck extends LinearOpMode implements AutonomousConstants, TeleOpConstants {

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


    public void implementDoubleCheck(int colorValue){


        if ( colorValue < 200) {

            this.assist.setPosition(0.1); // 'u' is the assist
            this.arm.setPower(0.3);
            sleep(AutonomousConstants.ARM_DROP_DISTANCE/16); // if you want to change this, make sure you change it in AutonomousConstants
            this.arm.setPower(0);
            this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0);
            this.sleep(300);
            this.mecanumDrive.stopMoving();
            this.arm.setPower(-0.5);
            sleep(AutonomousConstants.ARM_DROP_DISTANCE/6); // if you want to change this, make sure you change it in AutonomousConstants
            this.arm.setPower(0);
            sleep(300);
            this.grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
            this.assist.setPosition(0.1); // 'u' is the assist
            sleep(150);
            this.mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
            sleep(200);
            this.mecanumDrive.stopMoving();
            this.assist.setPosition(1);
            sleep(1300);
        }




    }


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
