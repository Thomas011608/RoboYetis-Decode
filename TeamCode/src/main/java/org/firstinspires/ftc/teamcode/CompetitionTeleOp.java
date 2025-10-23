package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.Format;

@TeleOp(name="Competition TeleOp", group="Linear OpMode")
public class CompetitionTeleOp extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx greenLauncher = null;
    private DcMotorEx purpleLauncher = null;
    //private CRServo greenFeeder = null;
    //private CRServo purpleFeeder = null;
    //private CRServo purpleFeeder2 = null;
    private Servo greenFeeder = null;
    private Servo purpleFeeder = null;
    private Servo purpleFeeder2 = null;


    ElapsedTime feederTimer = new ElapsedTime();
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 1075;
    private enum LaunchStateGreen {
        IDLE_GREEN,
        SPIN_UP_GREEN,
        LAUNCH_GREEN,
        LAUNCHING_GREEN,
    }
    private enum LaunchStatePurple {
        IDLE_PURPLE,
        SPIN_UP_PURPLE,
        LAUNCH_PURPLE,
        LAUNCHING_PURPLE,
    }

    private LaunchStateGreen launchStateGreen;
    private LaunchStatePurple launchStatePurple;

    @Override
    public void runOpMode() {
        launchStatePurple = LaunchStatePurple.IDLE_PURPLE;
        launchStateGreen = LaunchStateGreen.IDLE_GREEN;


        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // initialize launcher hardware variables
        greenLauncher = hardwareMap.get(DcMotorEx.class,"green_launcher");
        purpleLauncher = hardwareMap.get(DcMotorEx.class,"purple_launcher");

        //Initialize servo hardware variables
        /*greenFeeder = hardwareMap.get(CRServo.class,"green_feeder");
        purpleFeeder = hardwareMap.get(CRServo.class,"purple_feeder");
        purpleFeeder2 = hardwareMap.get(CRServo.class,"purple_feeder_two");
        greenFeeder.setPower(STOP_SPEED);
        purpleFeeder.setPower(STOP_SPEED);
        purpleFeeder2.setPower(STOP_SPEED);*/
        greenFeeder = hardwareMap.get(Servo.class,"green_feeder");
        purpleFeeder = hardwareMap.get(Servo.class,"purple_feeder");
        purpleFeeder2 = hardwareMap.get(Servo.class,"purple_feeder_two");
        greenFeeder.setPosition(STOP_SPEED);
        purpleFeeder.setPosition(STOP_SPEED);
        purpleFeeder2.setPosition(STOP_SPEED);

        //Set Driving Direction
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Set Launcher Direction
        greenLauncher.setDirection(DcMotor.Direction.REVERSE);
        purpleLauncher.setDirection(DcMotor.Direction.FORWARD);

        greenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        purpleLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        greenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        purpleLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));

        //Set Driving Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set Launcher Zero Power Behavior
        greenLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        purpleLauncher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }



            // Send calculated power to wheels
            frontLeftDrive.setPower(frontLeftPower);
            frontRightDrive.setPower(frontRightPower);
            backLeftDrive.setPower(backLeftPower);
            backRightDrive.setPower(backRightPower);


            if (gamepad1.y) {
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.b) { // stop flywheel
                greenLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad1.a) {
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
            } else if (gamepad1.x) { // stop flywheel
                purpleLauncher.setVelocity(STOP_SPEED);
            }

            launchPurple(gamepad1.leftBumperWasPressed());
            launchGreen(gamepad1.rightBumperWasPressed());

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("State", launchStatePurple);
            telemetry.addData("motorSpeed", purpleLauncher.getVelocity());
            telemetry.addData("State", launchStateGreen);
            telemetry.addData("motorSpeed", greenLauncher.getVelocity());

            telemetry.update();
        }
    }
    void launchPurple(boolean shotRequested) {
        switch (launchStatePurple) {
            case IDLE_PURPLE:
                if (shotRequested) {
                    launchStatePurple = LaunchStatePurple.SPIN_UP_PURPLE;
                }
                break;
            case SPIN_UP_PURPLE:
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (purpleLauncher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchStatePurple = LaunchStatePurple.LAUNCH_PURPLE;
                }
                break;
            case LAUNCH_PURPLE:
                //purpleFeeder.setPower(FULL_SPEED);
                //purpleFeeder2.setPower(FULL_SPEED);
                purpleFeeder.setPosition(FULL_SPEED);
                purpleFeeder2.setPosition(FULL_SPEED);
                feederTimer.reset();
                launchStatePurple = LaunchStatePurple.LAUNCHING_PURPLE;
                break;
            case LAUNCHING_PURPLE:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchStatePurple = LaunchStatePurple.IDLE_PURPLE;
                    purpleFeeder.setPosition(STOP_SPEED);
                    purpleFeeder2.setPosition(STOP_SPEED);
                }
                break;
        }
    }
    void launchGreen(boolean shotRequested) {
        switch (launchStateGreen) {
            case IDLE_GREEN:
                if (shotRequested) {
                    launchStateGreen = LaunchStateGreen.SPIN_UP_GREEN;
                }
                break;
            case SPIN_UP_GREEN:
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (greenLauncher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchStateGreen = LaunchStateGreen.LAUNCH_GREEN;
                }
                break;
            case LAUNCH_GREEN:
                //greenFeeder.setPower(FULL_SPEED);
                greenFeeder.setPosition(FULL_SPEED);
                feederTimer.reset();
                launchStateGreen = LaunchStateGreen.LAUNCHING_GREEN;
                break;
            case LAUNCHING_GREEN:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchStateGreen = LaunchStateGreen.IDLE_GREEN;
                    //greenFeeder.setPower(STOP_SPEED);
                    greenFeeder.setPosition(STOP_SPEED);
                }
                break;
        }
    }
}
