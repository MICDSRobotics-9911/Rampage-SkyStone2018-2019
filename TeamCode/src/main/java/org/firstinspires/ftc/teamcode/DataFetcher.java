package org.firstinspires.ftc.teamcode;

import android.content.Context;
import android.util.Log;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.autonomii.perceptron.SSPPoint;
import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.ClampState;
import org.firstinspires.ftc.teamcode.lib.GrabberState;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

import java.io.IOException;
import java.io.OutputStreamWriter;
import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "DataFetcher", group = "Basic")
public class DataFetcher extends OpMode implements TeleOpConstants, AutonomousConstants {
    private Robot robot;
    private MecanumDrive mecanumDrive;
    private MotorPair intake;
    private DcMotor arm;
    private DcMotor elevator;
    private Servo grabber;
    private Servo assist;
    private Servo clampLeft;
    private Servo clampRight;
    private GrabberState grabberState = GrabberState.CLOSED;
    private ClampState clampState = ClampState.UP;
    private IMUWrapper imuWrapper;
    private IntegratingGyroscope gyro;
    private ModernRoboticsI2cGyro modernRoboticsI2cGyro;
    private AngularVelocity rates;
    private List<SSPPoint> dataPoints = new ArrayList<>();

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.imuWrapper = new IMUWrapper(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");
        this.assist = hardwareMap.get(Servo.class, "assist");
        this.clampLeft = hardwareMap.get(Servo.class, "clamp_left");
        this.clampRight = hardwareMap.get(Servo.class, "clamp_right");


        this.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        modernRoboticsI2cGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope)modernRoboticsI2cGyro;
        modernRoboticsI2cGyro.calibrate();
    }

    public float accelerationMagnitiude(Acceleration a) {
        return (float) Math.abs(Math.sqrt(Math.pow(a.xAccel,2) + Math.pow(a.yAccel,2) + Math.pow(a.zAccel,2)));
    }

    @Override
    public void loop() {
        //this.imuWrapper.updateAngles();
        /*telemetry.addData("Acceleration", this.imuWrapper.getIMU().getAcceleration());
        telemetry.addData("Heading", this.imuWrapper.getHeading());*/
        //Log.i("[DF]", accelerationMagnitiude(this.imuWrapper.getIMU().getAcceleration()) + "," + this.imuWrapper.getHeading());
        //this.dataPoints.add(new SSPPoint(this.deltaAngleMagnititude(), this.imuWrapper.getHeading()));
        telemetry.addData("isGrabberOpen", this.grabberState);
        telemetry.addData("Linear accel x", this.imuWrapper.getIMU().getLinearAcceleration().xAccel);
        telemetry.addData("Linear accel y", this.imuWrapper.getIMU().getLinearAcceleration().yAccel);
        telemetry.addData("flux x:", this.imuWrapper.getIMU().getMagneticFieldStrength().x);
        telemetry.addData("flux y:", this.imuWrapper.getIMU().getMagneticFieldStrength().y);
        telemetry.addData("flux z:", this.imuWrapper.getIMU().getMagneticFieldStrength().z);
        this.dataPoints.add(new SSPPoint(this.linearAccelerationMagnitude(), modernRoboticsI2cGyro.getHeading()));
        telemetry.addData("DP Heap:", this.dataPoints.size());
        telemetry.update();

        this.mecanumDrive.complexDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        // assist
        this.assist.setPosition((gamepad1.right_bumper || gamepad2.right_bumper) ? TeleOpConstants.ASSIST_CLOSED : TeleOpConstants.ASSIST_OPEN);

        // grabber
        if (this.grabberState.equals(GrabberState.CLOSED) && (gamepad2.right_bumper || gamepad1.right_bumper)) {
            // we want to open it partially then
            this.grabberState = GrabberState.PARTIAL;
        }
        else if (this.grabberState.equals(GrabberState.PARTIAL) && !(gamepad2.right_bumper || gamepad1.right_bumper)) {
            // we want to close it by default then then
            this.grabberState = GrabberState.CLOSED;
        }
        else if (gamepad2.dpad_left || gamepad1.dpad_left) {
            // fully open the grabber
            this.grabberState = GrabberState.OPEN;
        }
        else if (this.grabberState.equals(GrabberState.OPEN) && (gamepad2.dpad_right || gamepad1.dpad_right)) {
            // close the grabber from the full position
            this.grabberState = GrabberState.CLOSED;
        }

        if (gamepad1.a) {
            try {
                OutputStreamWriter outputStreamWriter = new OutputStreamWriter(hardwareMap.appContext.openFileOutput("datapoints.txt", Context.MODE_PRIVATE));
                for (SSPPoint dp : this.dataPoints) {
                    outputStreamWriter.write(dp.toString());
                }
                outputStreamWriter.close();
                telemetry.addData("DW:" , "Done");
            }
            catch (IOException io) {
                telemetry.addData("Error", io.getMessage());
            }
        }

        // grabber servo enum controller switch
        switch (this.grabberState) {
            case OPEN:
                this.grabber.setPosition(TeleOpConstants.GRABBER_FULL_OPEN);
                break;
            case PARTIAL:
                this.grabber.setPosition(TeleOpConstants.GRABBER_PARTIAL_OPEN);
                break;
            case CLOSED:
                this.grabber.setPosition(TeleOpConstants.GRABBER_CLOSED);
                break;
        }

        // arm
        this.arm.setPower(gamepad2.left_stick_y * 0.5);

        // intake
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.x || gamepad2.x) ? 1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.x || gamepad2.x) ? -1 : 0);
        this.intake.getMotor1().setPower((this.intake.getMotor1().getPower() == 0) && (gamepad1.y || gamepad2.y) ? -1 : 0);
        this.intake.getMotor2().setPower((this.intake.getMotor2().getPower() == 0) && (gamepad1.y || gamepad2.y) ? 1 : 0);

        // elevator
        this.elevator.setPower(gamepad2.right_stick_y);
        /*this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.dpad_right) ? 1 : 0);
        this.elevator.setPower((this.elevator.getPower() == 0 && gamepad1.left_bumper) ? -1 : 0);*/

        // clamp
        if (this.clampState.equals(ClampState.UP) && (gamepad2.dpad_down || gamepad1.dpad_down)) {
            // close the clamp
            this.clampState = ClampState.DOWN;
        }
        else if (this.clampState.equals(ClampState.DOWN) && (gamepad2.dpad_up || gamepad1.dpad_up)) {
            // raise the clamp
            this.clampState = ClampState.UP;
        }

        // clamp servo switch controller
        switch (this.clampState) {
            case UP:
                this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_UP);
                this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_UP);
                break;
            case DOWN:
                this.clampLeft.setPosition(AutonomousConstants.CLAMP_LEFT_DOWN);
                this.clampRight.setPosition(AutonomousConstants.CLAMP_RIGHT_DOWN);
                break;
        }

        // stop
        if (gamepad1.right_bumper) {
            this.intake.stopMoving();
            this.mecanumDrive.stopMoving();
            //this.grabber.setPower(0);
            this.arm.setPower(0);
            this.elevator.setPower(0);
        }
    }

    public float deltaAngleMagnititude() {
        AngularVelocity rates = gyro.getAngularVelocity(AngleUnit.DEGREES);
        return (float)Math.sqrt(Math.pow(rates.xRotationRate - 0.5,2) + Math.pow(rates.yRotationRate - 0.5,2));
    }

    public float linearAccelerationMagnitude() {
        Acceleration current = this.imuWrapper.getIMU().getLinearAcceleration();
        return (float) Math.sqrt(Math.pow(current.xAccel,2) + Math.pow(current.yAccel,2));
    }
}
