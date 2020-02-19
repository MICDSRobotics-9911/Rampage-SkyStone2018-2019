package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "MotorTest")
@Disabled
public class MotorTest extends OpMode {

    private DcMotor arm;

    @Override
    public void init() {
        this.arm = hardwareMap.get(DcMotor.class, "elevator");
    }

    @Override
    public void loop() {
        this.arm.setPower(gamepad1.right_stick_y);
    }
}
