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


@TeleOp(name="Competition TeleOp", group="Linear OpMode")
public class CompetitionTeleOp extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private CRServo greenFeeder = null;
    private CRServo purpleFeeder = null;
    private DcMotor intake = null;


    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    final double INTAKE_TIME_SECONDS = 1.0;
    final double FEED_TIME_SECONDS = 0.4; //The feeder servos run this long when a shot is requested.
    final double MAX_SPEED = 1.0; //We send this power to the servos when we want them to stop.
    final double HOLD_SPEED = 0.6;
    final double STOP_SPEED = 0.0;
    final double LAUNCHER_TARGET_VELOCITY_FAST = 850;
    final double LAUNCHER_MIN_VELOCITY_FAST = 800;
    final double LAUNCHER_TARGET_VELOCITY_SLOW = 675;
    final double LAUNCHER_MIN_VELOCITY_SLOW = 625;
    final double DRIVING_SPEED_MULTIPLIER = 0.8;
    private enum LaunchState {
        IDLE,
        SPIN_UP_FAST,
        SPIN_UP_SLOW,
        LAUNCH,
        SPIN_DOWN,
    }
    private enum IntakeState {
        IDLE,
        INTAKE,
        HOLD,
    }
    private LaunchState launchState;
    private IntakeState intakeState;

    @Override
    public void runOpMode() {
        launchState = LaunchState.IDLE;
        intakeState = IntakeState.IDLE;

        // TODO: Driving Motor Definitions
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        //Set Driving Direction
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.REVERSE);
        //Set Driving Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Launcher Motor Definitions
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        //Set Launcher Direction
        launcher.setDirection(DcMotor.Direction.FORWARD);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));
        //Set Launcher Zero Power Behavior
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // TODO: Feeder Servo Definitions
        greenFeeder = hardwareMap.get(CRServo.class,"green_feeder");
        purpleFeeder = hardwareMap.get(CRServo.class,"purple_feeder");
        greenFeeder.setPower(STOP_SPEED);
        purpleFeeder.setPower(STOP_SPEED);
        //Set Servo Direction
        greenFeeder.setDirection(CRServo.Direction.FORWARD);
        purpleFeeder.setDirection(CRServo.Direction.REVERSE);

        // TODO: Intake Motor Definitions
        intake = hardwareMap.get(DcMotor.class,"intake");
        intake.setDirection(DcMotor.Direction.FORWARD);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

            intakeBall(gamepad2.dpadUpWasPressed(), gamepad2.dpadDownWasPressed());

            launch(gamepad2.xWasPressed(), gamepad2.aWasPressed(), gamepad2.bWasPressed(), gamepad2.yWasPressed());

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());

            telemetry.addData("State", launchState);
            telemetry.addData("motorSpeed", launcher.getVelocity());

            telemetry.update();
        }
    }
    void intakeBall(boolean startIntake, boolean stopIntake){
        switch (intakeState) {
            case IDLE:
                if (startIntake){
                    intakeState = IntakeState.INTAKE;
                }
                break;
            case INTAKE:
                intake.setPower(MAX_SPEED);
                intakeTimer.reset();
                if (intakeTimer.seconds() > INTAKE_TIME_SECONDS){
                    intakeState = IntakeState.HOLD;
                }
                if (stopIntake){
                    intake.setPower(STOP_SPEED);
                    intakeState = IntakeState.IDLE;
                }
                break;
            case HOLD:
                intake.setPower(HOLD_SPEED);
                if (startIntake){
                    intakeState = IntakeState.INTAKE;
                }
                if (stopIntake){
                    intake.setPower(STOP_SPEED);
                    intakeState = IntakeState.IDLE;
                }
                break;
        }
    }
    void launch(boolean slowShotRequested, boolean fastShotRequested, boolean greenShotRequested, boolean purpleShotRequested){
        switch (launchState) {
            case IDLE:
                if (fastShotRequested) {
                    launchState = LaunchState.SPIN_UP_FAST;
                }
                if (slowShotRequested) {
                    launchState = LaunchState.SPIN_UP_SLOW;
                }
                break;
            case SPIN_UP_FAST:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_FAST);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY_FAST) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case SPIN_UP_SLOW:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY_SLOW) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                if (purpleShotRequested){
                    purpleFeeder.setPower(MAX_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.SPIN_DOWN;
                }
                if (greenShotRequested){
                    greenFeeder.setPower(MAX_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.SPIN_DOWN;
                }
                break;
            case SPIN_DOWN:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launcher.setVelocity(STOP_SPEED);
                    greenFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
