package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {
    private Servo clamp;

    public void runOpMode() {
        this.clamp = hardwareMap.get(Servo.class, "clamp");

        waitForStart();

        this.clamp.setPosition(0.5); // up position
        sleep(5000);
        this.clamp.setPosition(0); // down position
        sleep(5000);
    }
}
