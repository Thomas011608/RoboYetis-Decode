package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@TeleOp(name = "CompetitionTeleOp", group = "Linear OpMode")
public class CompetitionTeleOp extends LinearOpMode {

    // HEADER: Declare OpMode members for each of the motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    private DcMotor leftFeeder = null;
    private DcMotor rightFeeder = null;
    private DcMotor intake = null;
    private Servo SortPaddle = null;
    NormalizedColorSensor colorSensor;
    private HuskyLens huskyLens; //huskyLens is the variable of our camera

    // HEADER: Defining timers
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime launchTimer = new ElapsedTime();

    // HEADER: Defining final variables
    final double LAUNCH_TIME_SECONDS = 5.0; //The maximum time that the launcher is on for
    final int POSITION_ALIGNMENT_PIXELS = 15; // The range (+- this amount) of pixels the tag can be when aligned with the goal.
    final double FEED_TIME_SECONDS = 1.0; //The feeder servos run this long when a shot is requested.
    final double MAX_SPEED = 1.0; //We send this power to the servos when we want them to stop.
    final double MAX_SPEED_REVERSE = -1.0;
    final double HOLD_SPEED = 0.6;
    final double STOP_SPEED = 0.0;
    final double leftSort = 0.0;
    final double idleSort = 0.5;
    final double rightSort = 1.0;
    final double LAUNCHER_TARGET_VELOCITY_FAST = 1400;
    final double LAUNCHER_TARGET_VELOCITY_SLOW = 1200;
    final double DRIVING_SPEED_MULTIPLIER = 0.8;
    final float GAIN = 12;
    private enum LaunchState{
        IDLE,
        QUIT,
        LEFTLAUNCH,
        RIGHTLAUNCH,
        LEFTLAUNCHED,
        RIGHTLAUNCHED,
    }
    private enum IntakeState {
        IDLE,
        INTAKE,
        OUTTAKE
    }

    // HEADER: Define other variables
    private int GoalID = 0;
    private boolean AdaptiveLaunchSpeed = true;
    private String launchState = "Idle";
    private IntakeState intakeState = IntakeState.IDLE;

    @Override
    public void runOpMode() {

        // HEADER: Get the Alliance data from Gamepad 1
        telemetry.addData("Alliance", "Gamepad 1 press D-Pad Up for Red, D-Pad Down for Blue");
        telemetry.update();
        while (!opModeIsActive()) {
            if (gamepad1.dpad_up){
                GoalID = 4;
                telemetry.addData("Alliance","Red");
                break;
            }
            if (gamepad1.dpad_down) {
                GoalID = 5;
                telemetry.addData("Alliance","Blue");
                break;
            }
        }

        // HEADER: Driving Motor Definitions
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        //Set Driving Direction
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
        //Set Driving Zero Power Behavior
        frontLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HEADER: Launcher Motor Definitions
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        //Set Launcher Direction
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));
        //Set Launcher Zero Power Behavior
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HEADER: Feeder Servo Definitions
        leftFeeder = hardwareMap.get(DcMotor.class, "left_feeder");
        rightFeeder = hardwareMap.get(DcMotor.class, "right_feeder");
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        //Set Servo Direction
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);

        // HEADER: Gate Servo Definitions
        SortPaddle = hardwareMap.get(Servo.class, "sorting_gate");
        SortPaddle.setPosition(idleSort);

        // HEADER: Intake Motor Definitions
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HEADER: Color Sensor Definitions
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        final float[] hsvValues = new float[3];

        // HEADER: Camera Definitions and Initialization
        huskyLens = hardwareMap.get(HuskyLens.class, "camera");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens", "Not Initialized");
        }

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // HEADER: defining math for driving using mecanum wheels
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
            frontLeftDrive.setPower(frontLeftPower * DRIVING_SPEED_MULTIPLIER);
            frontRightDrive.setPower(frontRightPower * DRIVING_SPEED_MULTIPLIER);
            backLeftDrive.setPower(backLeftPower * DRIVING_SPEED_MULTIPLIER);
            backRightDrive.setPower(backRightPower * DRIVING_SPEED_MULTIPLIER);

            // HEADER: Set launch speed and intake state
            //Set Adaptive Launcher Speed (Right Bumper)
            if (gamepad2.rightBumperWasPressed()) {
                AdaptiveLaunchSpeed = !AdaptiveLaunchSpeed;
            }
            telemetry.addData("Adaptive Launch Speed", AdaptiveLaunchSpeed);

            /*
            // HEADER: Set values fo the color sensor and set it up to get data
            colorSensor.setGain(GAIN);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);
             */

            // HEADER: Call various functions used for intake and launching
            intakeBall (gamepad2.dpadDownWasPressed(), gamepad2.leftBumperWasPressed());
            launchState = launch(gamepad2.yWasPressed(), gamepad2.xWasPressed(), gamepad2.bWasPressed(), AdaptiveLaunchSpeed);
            sort(gamepad2.dpadLeftWasPressed(), gamepad2.dpadUpWasPressed(), gamepad2.dpadRightWasPressed());

            // HEADER: Use telemetry to print any desired information to the driver hub
            telemetry.addData("Launch State", launchState);
            telemetry.addData("Intake State", intakeState);
            telemetry.addData("Motor Speed", launcher.getVelocity());

            telemetry.update();
        }
    }

    //HEADER: sort() function
    void sort(boolean Left, boolean Idle, boolean Right) {
        // Set sorting servo to neutral (D-Pad Up).
        if (Idle) {
            SortPaddle.setPosition(idleSort);
        }

        // Set sorting servo to right (D-Pad Right)
        if (Right) {
            SortPaddle.setPosition(rightSort);
        }

        // Set sorting servo to left (D-Pad Left)
        if (Left) {
            SortPaddle.setPosition(leftSort);
        }
    }

    // HEADER: intakeBall() function
    void intakeBall(boolean in, boolean out){
        switch (intakeState) {
            case IDLE: {
                intake.setPower(STOP_SPEED);
                if(in) {
                    intakeState = IntakeState.INTAKE;
                }
                if(out) {
                    intakeState = IntakeState.OUTTAKE;
                }
                break;
            }
            case INTAKE: {
                intake.setPower(MAX_SPEED);
                if(in) {
                    intakeState = IntakeState.IDLE;
                }
                if(out){
                    intake.setPower(STOP_SPEED);
                    intakeState = IntakeState.OUTTAKE;
                }
                break;

            }
            case OUTTAKE: {
                intake.setPower(MAX_SPEED_REVERSE);
                if(in){
                    intake.setPower(STOP_SPEED);
                    intakeState = IntakeState.INTAKE;
                }
                if(out){
                    intakeState = IntakeState.IDLE;
                }
                break;
            }
        }

        /*
        if (intakeState) {
            intake.setPower(MAX_SPEED);
        } else if (intakeBack) {
            intake.setPower(MAX_SPEED_REVERSE);
        } else {
            intake.setPower(STOP_SPEED);
        }
         */
    }

    // HEADER: launch() function
    String launch(boolean shotRequested, boolean leftShotRequested,boolean rightShotRequested, boolean adaptive) {
        double distance = getDistance();
        double power;

        if (adaptive) {
            if (distance < 130) {
                power = 1300;
            } else if (distance > 240) {
                power = 1475;
            } else {
                power = 0.0360562 * (Math.pow(distance, 2)) - 11.25698 * distance + 2092.27902;
            }
        } else {
            if (distance < 240) {
                power = LAUNCHER_TARGET_VELOCITY_SLOW;
            } else {
                power = LAUNCHER_TARGET_VELOCITY_FAST;
            }
        }
        double minPower = power - 100;
        telemetry.addData("Power", power);

        if (shotRequested) {
            intakeState = IntakeState.IDLE;
            //intakeBall(false,false);

            launcher.setVelocity(power);
            launchTimer.reset();
            return "Spin Up";
        }

        if (rightShotRequested && launcher.getVelocity() > minPower) {
            rightFeeder.setPower(MAX_SPEED);
            feederTimer.reset();
            return "Right Shot";
        }

        if (leftShotRequested && launcher.getVelocity() > minPower) {
            leftFeeder.setPower(MAX_SPEED);
            feederTimer.reset();
            return "Left Shot";
        }

        if (feederTimer.seconds() >= FEED_TIME_SECONDS && launchTimer.seconds() >= LAUNCH_TIME_SECONDS) {
            leftFeeder.setPower(STOP_SPEED);
            rightFeeder.setPower(STOP_SPEED);
            launcher.setVelocity(STOP_SPEED);
            launcher.setPower(STOP_SPEED);
            return "Idle";
        }
        return "Idle";
    }

    // HEADER: getDistance() function
    double getDistance() {
        double distance = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            if (block.id == GoalID) {
                double area = block.width * block.height;
                distance = Math.pow((area / 16139259.8), (1 / -1.89076));

                if (block.x > 160-POSITION_ALIGNMENT_PIXELS && block.x < 160 + POSITION_ALIGNMENT_PIXELS && launcher.getVelocity() == 0) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                }
            }
        }
        telemetry.addData("Distance", distance);
        return distance;
    }
}