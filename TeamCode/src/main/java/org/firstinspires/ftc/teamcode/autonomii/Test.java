package org.firstinspires.ftc.teamcode.autonomii;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@Autonomous(name = "Test")
public class Test extends LinearOpMode {
    private Servo clamp;
    private DigitalChannel frontSwitch;

    public void runOpMode() {
        this.clamp = hardwareMap.get(Servo.class, "clamp");
        this.frontSwitch = hardwareMap.get(DigitalChannel.class, "front_switch");

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("switch", this.frontSwitch.getState());
            telemetry.update();
        }

        /*this.clamp.setPosition(0.5); // up position
        sleep(5000);
        this.clamp.setPosition(0); // down position
        sleep(5000);*/
    }
}

