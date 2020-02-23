package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@Autonomous(name = "capstoneControl", group = "Generic")
public class capstoneControl extends LinearOpMode implements AutonomousConstants, TeleOpConstants {

    private MotorPair intake;

    public void runOpMode() {

        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");

        while (opModeIsActive()) {

            this.intake.getMotor1().setPower(1);
            this.intake.getMotor2().setPower(-1);
            sleep(1000);
            this.intake.getMotor1().setPower(0);
            this.intake.getMotor2().setPower(0);
        }
    }
}
