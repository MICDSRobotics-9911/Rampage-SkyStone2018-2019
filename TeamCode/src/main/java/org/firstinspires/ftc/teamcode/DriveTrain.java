package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;

import org.firstinspires.ftc.micdsrobotics.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.micdsrobotics.robotplus.hardware.Robot;

public class DriveTrain extends Robot<MecanumDrive> {

    @Override
    public void initHardware(HardwareMap hardwareMap) {
    }

    public void loop() {

    }

    @Override
    public double voltageToDistance(double voltage) {

        return 0;
    }
}
