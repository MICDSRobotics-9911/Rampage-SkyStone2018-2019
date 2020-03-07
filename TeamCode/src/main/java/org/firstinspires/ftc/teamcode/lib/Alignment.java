package org.firstinspires.ftc.teamcode.lib;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.ODSasTouchSensor;

public class Alignment {
    public static void alignToWall(LinearOpMode lop, ODSasTouchSensor frontODS, MecanumDrive mecanumDrive, double voltage) {
        mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -1, 0);
        lop.sleep(450);
        mecanumDrive.stopMoving();
        while (!frontODS.isPressed()) {
            mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), -0.5, 0);
            lop.sleep(1);
        }
        mecanumDrive.stopMoving();
        lop.sleep(100);
        mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        lop.sleep(TimeOffsetVoltage.calculateDistance(voltage, 44));
        mecanumDrive.stopMoving();
    }

    public static void alignToWallNoODS(LinearOpMode lop, MecanumDrive mecanumDrive, double voltage) {
        mecanumDrive.stopMoving();
        lop.sleep(100);
        mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0);
        lop.sleep(TimeOffsetVoltage.calculateDistance(voltage, 44));
        mecanumDrive.stopMoving();
    }
}
