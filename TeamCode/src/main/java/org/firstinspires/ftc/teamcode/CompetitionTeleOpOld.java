package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@TeleOp(name="Competition TeleOp", group="Linear OpMode")
public class CompetitionTeleOpOld extends LinearOpMode {
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
    //private Servo purpleFeeder2 = null;


    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime launchTimer = new ElapsedTime();
    final double FEED_TIME_SECONDS = 0.4; //The feeder servos run this long when a shot is requested.
    final double LAUNCH_SECONDS = 1.0;
    final double DOWN_POSITION = 0.0; //We send this power to the servos when we want them to stop.
    final double UP_POSITION = 0.2;
    final double STOP_SPEED = 0.0;
    final double MAX_SPEED = 2000;
    final double LAUNCHER_TARGET_VELOCITY_FAST = 850;
    final double LAUNCHER_MIN_VELOCITY_FAST = 800;
    final double LAUNCHER_TARGET_VELOCITY_SLOW = 675;
    final double LAUNCHER_MIN_VELOCITY_SLOW = 625;
    final double LAUNCHER_INTAKE_VELOCITY = -500;
    final double DRIVING_SPEED_MULTIPLIER = 1.0;
    private enum LaunchState {
        IDLE,
        SPIN_UP_PURPLE_FAST,
        SPIN_UP_PURPLE_SLOW,
        LAUNCH_PURPLE,
        SPIN_DOWN_PURPLE,
        SPIN_UP_GREEN_FAST,
        SPIN_UP_GREEN_SLOW,
        LAUNCH_GREEN,
        SPIN_DOWN_GREEN,
    }
    private LaunchState launchState;

    @Override
    public void runOpMode() {
        launchState = LaunchState.IDLE;


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
        greenFeeder.setPosition(UP_POSITION);
        purpleFeeder.setPosition(DOWN_POSITION);
        //purpleFeeder2 = hardwareMap.get(Servo.class,"purple_feeder_two");
        //greenFeeder.setPosition(STOP_SPEED);
        //purpleFeeder.setPosition(STOP_SPEED);
        //purpleFeeder2.setPosition(STOP_SPEED);

        //Set Driving Direction
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);

        //Set Launcher Direction
        greenLauncher.setDirection(DcMotor.Direction.FORWARD);
        purpleLauncher.setDirection(DcMotor.Direction.REVERSE);

        greenLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        purpleLauncher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        greenLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));
        purpleLauncher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));

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
            frontLeftDrive.setPower(frontLeftPower*DRIVING_SPEED_MULTIPLIER);
            frontRightDrive.setPower(frontRightPower*DRIVING_SPEED_MULTIPLIER);
            backLeftDrive.setPower(backLeftPower*DRIVING_SPEED_MULTIPLIER);
            backRightDrive.setPower(backRightPower*DRIVING_SPEED_MULTIPLIER);

            /*if (gamepad2.dpad_right) {
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_FAST);
                sleep(1500);
                greenFeeder.setPosition(DOWN_POSITION);
                sleep(400);
                greenFeeder.setPosition(UP_POSITION);
                greenLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad2.dpad_left) {
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_FAST);
                sleep(1500);
                purpleFeeder.setPosition(UP_POSITION);
                sleep(400);
                purpleFeeder.setPosition(DOWN_POSITION);
                purpleLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad2.b) {
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
                sleep(1500);
                greenFeeder.setPosition(DOWN_POSITION);
                sleep(400);
                greenFeeder.setPosition(UP_POSITION);
                greenLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad2.x) {
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
                sleep(1500);
                purpleFeeder.setPosition(UP_POSITION);
                sleep(400);
                purpleFeeder.setPosition(DOWN_POSITION);
                purpleLauncher.setVelocity(STOP_SPEED);
            }*/

            /*if (gamepad2.x) {
                greenLauncher.setVelocity(MAX_SPEED);
            } else if (gamepad2.b) { // stop flywheel
                greenLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad2.dpad_left) {
                purpleLauncher.setVelocity(MAX_SPEED);
            } else if (gamepad2.dpad_right) { // stop flywheel
                purpleLauncher.setVelocity(STOP_SPEED);
            }*/
            /*if (gamepad1.dpad_left) {
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
            } else if (gamepad1.a) { // stop flywheel
                greenLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad1.dpad_right) {
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
            } else if (gamepad1.a) { // stop flywheel
                purpleLauncher.setVelocity(STOP_SPEED);
            }*/

            /*if (gamepad2.y) {
                greenFeeder.setPosition(DOWN_POSITION);
            } else if (gamepad2.a) { // stop flywheel
                greenFeeder.setPosition(UP_POSITION);
            }
            if (gamepad2.dpad_down) {
                purpleFeeder.setPosition(DOWN_POSITION);
            } else if (gamepad2.dpad_up) { // stop flywheel
                purpleFeeder.setPosition(UP_POSITION);
            }*/

            if (gamepad2.leftBumperWasPressed()) {
                purpleLauncher.setVelocity(LAUNCHER_INTAKE_VELOCITY);
                sleep(750);
                purpleLauncher.setVelocity(STOP_SPEED);
            }
            if (gamepad2.rightBumperWasPressed()) {
                greenLauncher.setVelocity(LAUNCHER_INTAKE_VELOCITY);
                sleep(750);
                greenLauncher.setVelocity(STOP_SPEED);
            }

            launch(gamepad2.xWasPressed(), gamepad2.dpadLeftWasPressed(), gamepad2.bWasPressed(), gamepad2.dpadRightWasPressed());

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("State", launchState);
            telemetry.addData("motorSpeed", purpleLauncher.getVelocity());
            telemetry.addData("motorSpeed", greenLauncher.getVelocity());

            telemetry.update();
        }
    }
    void launch(boolean slowShotRequestedPurple, boolean fastShotRequestedPurple, boolean slowShotRequestedGreen, boolean fastShotRequestedGreen){
        switch (launchState) {
            case IDLE:
                if (fastShotRequestedPurple) {
                    launchState = LaunchState.SPIN_UP_PURPLE_FAST;
                }
                if (slowShotRequestedPurple) {
                    launchState = LaunchState.SPIN_UP_PURPLE_SLOW;
                }
                if (fastShotRequestedGreen) {
                    launchState = LaunchState.SPIN_UP_GREEN_FAST;
                }
                if (slowShotRequestedGreen) {
                    launchState = LaunchState.SPIN_UP_GREEN_SLOW;
                }
                break;
            case SPIN_UP_PURPLE_FAST:
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_FAST);
                if (purpleLauncher.getVelocity() > LAUNCHER_MIN_VELOCITY_FAST) {
                    launchState = LaunchState.LAUNCH_PURPLE;
                }
                break;
            case SPIN_UP_PURPLE_SLOW:
                purpleLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
                if (purpleLauncher.getVelocity() > LAUNCHER_MIN_VELOCITY_SLOW) {
                    launchState = LaunchState.LAUNCH_PURPLE;
                }
                break;
            case LAUNCH_PURPLE:
                purpleFeeder.setPosition(UP_POSITION);
                feederTimer.reset();
                launchState = LaunchState.SPIN_DOWN_PURPLE;
                break;
            case SPIN_DOWN_PURPLE:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    purpleLauncher.setVelocity(STOP_SPEED);
                    purpleFeeder.setPosition(DOWN_POSITION);
                    launchState = LaunchState.IDLE;
                }
                break;
            case SPIN_UP_GREEN_FAST:
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_FAST);
                if (greenLauncher.getVelocity() > LAUNCHER_MIN_VELOCITY_FAST) {
                    launchState = LaunchState.LAUNCH_GREEN;
                }
                break;
            case SPIN_UP_GREEN_SLOW:
                greenLauncher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
                if (greenLauncher.getVelocity() > LAUNCHER_MIN_VELOCITY_SLOW) {
                    launchState = LaunchState.LAUNCH_GREEN;
                }
                break;
            case LAUNCH_GREEN:
                greenFeeder.setPosition(DOWN_POSITION);
                feederTimer.reset();
                launchState = LaunchState.SPIN_DOWN_GREEN;
                break;
            case SPIN_DOWN_GREEN:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    greenLauncher.setVelocity(STOP_SPEED);
                    greenFeeder.setPosition(UP_POSITION);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
