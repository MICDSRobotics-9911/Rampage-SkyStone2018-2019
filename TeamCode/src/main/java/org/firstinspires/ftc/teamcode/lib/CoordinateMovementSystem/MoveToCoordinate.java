
package org.firstinspires.ftc.teamcode.lib.CoordinateMovementSystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import org.firstinspires.ftc.teamcode.robotplus.autonomous.TimeOffsetVoltage;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;


public class MoveToCoordinate { //declare class
    public static void check(int targetX, int targetY, TouchSensor front, TouchSensor back, TouchSensor left, TouchSensor right, LinearOpMode lop, MecanumDrive mecanumDrive, double voltage) { //declare method
        // we need a target coordinate, inputs from touch sensors on each side of the robot, a linearOpMode, MecanumDrive, and values for voltage


        lop.telemetry.addLine("The robot will now travel to (" + targetX + ", " + targetY + ").");

        int currentX = 0; // always reset the positioning variables to 0. This is the one for x values
        int currentY = 0; // always reset the positioning variables to 0. This is the one for y values


        while(!(Math.abs(targetX-currentX)<3)||!(Math.abs(targetY-currentY)<3)) { //begin loop to send robot on its way to target

            while (currentX < targetX) { //for as long as we need to go to the RIGHT
                mecanumDrive.complexDrive(MecanumDrive.Direction.RIGHT.angle(), 1, 0); //move to the RIGHT
                lop.sleep(TimeOffsetVoltage.calculateDistance(voltage, 1));  //only go a tiny precise distance
                currentX = currentX + 1; //update current variable values accordingly
                mecanumDrive.stopMoving(); //stop




            } //end while (RIGHT)


            while (currentX > targetX) { //for as long as we need to go to the LEFT
                mecanumDrive.complexDrive(MecanumDrive.Direction.LEFT.angle(), 1, 0); //move to the LEFT
                lop.sleep(TimeOffsetVoltage.calculateDistance(voltage, 1)); //only go a tiny precise distance
                currentX = currentX - 1; //update current variable values accordingly
                mecanumDrive.stopMoving(); //stop



            } //end while (LEFT)


            while (currentY < targetY) { //for as long as we need to go FORWARD
                mecanumDrive.complexDrive(MecanumDrive.Direction.UP.angle(), 1, 0); //move FORWARD
                lop.sleep(TimeOffsetVoltage.calculateDistance(voltage, 1)); //only go a tiny precise distance
                currentY = currentY + 1; //update current variable values accordingly
                mecanumDrive.stopMoving(); //stop



            } //end while (FORWARD)


            while (currentY > targetY) { //for as long as we need to go BACKWARD
                mecanumDrive.complexDrive(MecanumDrive.Direction.DOWN.angle(), 1, 0); //move BACKWARD
                lop.sleep(TimeOffsetVoltage.calculateDistance(voltage, 1)); //only go a tiny precise distance
                currentY = currentY - 1; //update current variable values accordingly
                mecanumDrive.stopMoving(); //stop



            } //end while (BACKWARD)


        }//mission accomplised. Robot is in position
        lop.telemetry.addLine(("Task Accomplished!"));
        lop.telemetry.addLine("The robot was arrived at/near (" + targetX + ", " + targetY + ").");
        lop.telemetry.addLine("Moving on to next set of instructions in main autonomous class...");
    } //end method
} // end class