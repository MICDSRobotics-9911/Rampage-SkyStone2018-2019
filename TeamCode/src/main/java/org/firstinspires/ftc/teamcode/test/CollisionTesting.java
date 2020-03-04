package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.teamcode.lib.AutonomousConstants;
import org.firstinspires.ftc.teamcode.lib.ClampState;
import org.firstinspires.ftc.teamcode.lib.GrabberState;
import org.firstinspires.ftc.teamcode.lib.TeleOpConstants;
import org.firstinspires.ftc.teamcode.lib.perceptron.CollisionExecutor;
import org.firstinspires.ftc.teamcode.robotplus.hardware.I2CGyroWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.IMUWrapper;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MecanumDrive;
import org.firstinspires.ftc.teamcode.robotplus.hardware.MotorPair;
import org.firstinspires.ftc.teamcode.robotplus.hardware.Robot;

@TeleOp(name = "Collision Testing", group = "Test")
// @Disabled
public class CollisionTesting extends OpMode implements TeleOpConstants, AutonomousConstants {
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
    public I2CGyroWrapper i2CGyroWrapper;
    private AngularVelocity rates;
    public IMUWrapper imuWrapper;
    public boolean crashed = false;
    private Thread collisionThread;

    @Override
    public void init() {
        this.robot = new Robot(hardwareMap);
        this.mecanumDrive = (MecanumDrive) this.robot.getDrivetrain();
        this.intake = new MotorPair(hardwareMap, "intake1", "intake2");
        this.arm = hardwareMap.get(DcMotor.class, "arm");
        this.elevator = hardwareMap.get(DcMotor.class, "elevator");
        this.grabber = hardwareMap.get(Servo.class, "grabber");
        this.assist = hardwareMap.get(Servo.class, "assist");
        this.clampLeft = hardwareMap.get(Servo.class, "clamp_left");
        this.clampRight = hardwareMap.get(Servo.class, "clamp_right");
        this.i2CGyroWrapper = new I2CGyroWrapper(hardwareMap);

        this.imuWrapper = new IMUWrapper(hardwareMap);

        this.elevator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // start the collision detector
        collisionThread = new Thread(new Runnable() {
            @Override
            public void run() {
                while (true) {
                    try {
                        // check for collision. If collided, stop to prevent further damage
                        if (CollisionExecutor.calculate(i2CGyroWrapper.getHeading(), imuWrapper)) {
                            crashed = true;
                        }
                        else {
                            crashed = false;
                        }
                        Thread.sleep(5);
                    }
                    catch (InterruptedException e) {}
                }
            }
        });
        collisionThread.start();
    }

    @Override
    public void loop() {
        telemetry.addData("Crashed", crashed);
        telemetry.update();

        this.mecanumDrive.complexDrive(-gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, telemetry);

        // stop
        if (gamepad1.right_bumper) {
            this.intake.stopMoving();
            this.mecanumDrive.stopMoving();
            //this.grabber.setPower(0);
            this.arm.setPower(0);
            this.elevator.setPower(0);
        }
    }
}
