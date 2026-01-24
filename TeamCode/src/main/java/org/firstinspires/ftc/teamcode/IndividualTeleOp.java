package org.firstinspires.ftc.teamcode;


import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.dfrobot.HuskyLens;

@Disabled
@TeleOp(name="IndividualTeleOp", group = "Test")
public class IndividualTeleOp extends LinearOpMode {
    // Declare OpMode members for each of the 4 motors.
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;
    private DcMotorEx launcher = null;
    //private CRServo leftFeeder = null;
    //private CRServo rightFeeder = null;
    private DcMotor leftFeeder = null;
    private DcMotor rightFeeder = null;
    private DcMotor intake = null;
    private Servo SortPaddle = null;
    NormalizedColorSensor colorSensor;
    private static HuskyLens huskyLens; //huskyLens is the variable of our camera

    // Create new variables for each action
    ElapsedTime feederTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime sortTimer = new ElapsedTime();
    ElapsedTime launchTimer = new ElapsedTime();
    ElapsedTime runtime = new ElapsedTime();
    final double INTAKE_TIME_SECONDS = 1.0; //The intake runs for this long before stopping
    final double FEED_TIME_SECONDS = 1.0; //The feeder servos run for this time when a shot is requested.
    final double FEED_IN_TIME_SECONDS = 1.0; // The feeder servos run backwards for this time
    final double LAUNCH_TIME_SECONDS = 1.0; // How long after speed up the launcher stays on.
    final double MAX_SPEED = 1.0;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double PurpleSort = 0.0;
    final double IdleSort = 0.5;
    final double GreenSort = 1.0;
    final double LAUNCHER_TARGET_VELOCITY_FAST = 1400;
    final double LAUNCHER_MIN_VELOCITY_FAST = 1500;
    final double LAUNCHER_TARGET_VELOCITY_SLOW = 1200;
    final double LAUNCHER_MIN_VELOCITY_SLOW = 1100;
    final double DRIVING_SPEED_MULTIPLIER = 0.8;
    final float GAIN = 12;
    boolean USE_POWER_FUNCTION = false;


    @Override
    public void runOpMode() {
        // HEADER: Reset Timers
        sortTimer.reset();
        feederTimer.reset();
        intakeTimer.reset();
        launchTimer.reset();

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
        launcher.setDirection(DcMotorEx.Direction.REVERSE);
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));
        //Set Launcher Zero Power Behavior
        launcher.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // HEADER: Feeder Servo Definitions
        //leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        //rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        leftFeeder = hardwareMap.get(DcMotor.class, "left_feeder");
        rightFeeder = hardwareMap.get(DcMotor.class, "right_feeder");
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        //Set Servo Direction
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);

        // HEADER: Gate Servo Definitions
        SortPaddle = hardwareMap.get(Servo.class, "sorting_gate");
        SortPaddle.setPosition(IdleSort);

        // HEADER: Intake Motor Definitions
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // HEADER: Color Sensor Definitions
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        final float[] hsvValues = new float[3];

        //HEADER: Camera Definitions and Initialization
        huskyLens = hardwareMap.get(HuskyLens.class, "camera");
        if (!huskyLens.knock()) {
            telemetry.addData("HuskyLens","Not Initialized");
            telemetry.update();
        }
        double distance = 0;



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

            // HEADER: Find AprilTags in the Camera's view
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
            HuskyLens.Block[] tags = huskyLens.blocks(); // tags is the value of AprilTags in view

            // HEADER: Set values fo the color sensor and set is up to get data
            colorSensor.setGain(GAIN);
            NormalizedRGBA colors = colorSensor.getNormalizedColors();
            Color.colorToHSV(colors.toColor(), hsvValues);

            //HEADER: Use Gamepad Buttons to control individual parts
            //Intake Wheel Forward (D-Pad Up)
            if (gamepad2.dpadUpWasPressed()) {
                telemetry.addData("Part:","Intake Forward");

                intake.setPower(MAX_SPEED);
                intakeTimer.reset();
            }
            if (intakeTimer.seconds() >= INTAKE_TIME_SECONDS) {
                intake.setPower(STOP_SPEED);
            }

            //Feeder Wheels Backward (D-Pad Down)
            if (gamepad2.dpadDownWasPressed()) {
                telemetry.addData("Part:","Feeder Backward");

                leftFeeder.setPower(-MAX_SPEED);
                rightFeeder.setPower(-MAX_SPEED);
                feederTimer.reset();
            }

            //Feeder Wheel Left Launch (D-Pad Left)
            if (gamepad2.dpadLeftWasPressed()) {
                telemetry.addData("Part:","Feed Left");

                leftFeeder.setPower(MAX_SPEED);
                feederTimer.reset();
            }

            //Feeder Wheel Right Launch (D-Pad Right)
            if (gamepad2.dpadRightWasPressed()) {
                telemetry.addData("Part:","Feed Right");

                rightFeeder.setPower(MAX_SPEED);
                feederTimer.reset();
            }

            if (feederTimer.seconds() >= FEED_IN_TIME_SECONDS) {
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
            }


            // Switch Power function (D-Pad Up)
            if (gamepad1.dpadUpWasPressed()){
                USE_POWER_FUNCTION = !USE_POWER_FUNCTION;
            }

            if (USE_POWER_FUNCTION) {
                if (GetDistance() != 0){
                    distance = GetDistance();
                }
                telemetry.addData("getDistance", distance);
                //double power = 0.117582*(Math.pow(distance,2)) - 37.19797*distance + 4136.25617;
                //double power = 0.0610355*(Math.pow(distance,2)) - 19.20547*distance + 2718.58097;
                double power;
                if (distance < 130) {
                    power = 1300;
                } else if (distance > 240) {
                    power = 1475;
                } else {
                    power = 0.0360562 * (Math.pow(distance, 2)) - 11.25698 * distance + 2092.27902;
                }
                if (gamepad2.xWasPressed()) {
                    launcher.setVelocity(power);
                    launchTimer.reset();
                }
                telemetry.addData("Power", power);
            } else {
                //Launcher Slow
                if (gamepad2.xWasPressed()) {
                    telemetry.addData("Part:", "Slow Launch");

                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_SLOW);
                    launchTimer.reset();
                }

                //Launcher Fast
                if (gamepad2.aWasPressed()) {
                    telemetry.addData("Part:", "Slow Launch");

                    launcher.setVelocity(LAUNCHER_TARGET_VELOCITY_FAST);
                    launchTimer.reset();
                }
                telemetry.addData("Slow Target", LAUNCHER_TARGET_VELOCITY_SLOW);
                telemetry.addData("Fast Target", LAUNCHER_TARGET_VELOCITY_FAST);
            }
            if (launchTimer.seconds() >= LAUNCH_TIME_SECONDS + 4) {
                launcher.setVelocity(STOP_SPEED);
                launcher.setPower(STOP_SPEED);
            }
            // Purple Sort Position (y)
            if (gamepad2.yWasPressed()) {
                telemetry.addData("Part:","Purple Sort");
                SortPaddle.setPosition(PurpleSort);

            }

            // Green Sort Position (b)
            if (gamepad2.bWasPressed()) {
                telemetry.addData("Part:","Green Sort");
                SortPaddle.setPosition(GreenSort);
            }


            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (HuskyLens.Block block : blocks) {
                telemetry.addData("Block", block.toString());

                if (block.x > 140 && block.x < 180) {
                    gamepad1.rumble(100);
                    gamepad2.rumble(100);
                }

            }


            // HEADER: Use telemetry to print any desired information to the driver hub
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());


            // Show color values found by the color sensor
            telemetry.addData("Color RGB", colors);

            //Display the motor speed
            telemetry.addData("motorSpeed", launcher.getVelocity());
            telemetry.update();
        }
        telemetry.clearAll();
        telemetry.addData("Status:", "Stopped");
        telemetry.update();
    }
    public static double GetDistance(){
        double distance = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            double area = block.width * block.height;
            distance = Math.pow((area / 16139259.8), (1 / -1.89076));
        }
        return distance;
    }
}


    /*
    void sort(double red, double green, double blue){
        double max;
        max = red+green+blue;
        if (red > 0.2 && red>green && blue>green && max<2.25) {
            telemetry.addData("Color","Purple");
            purpleGate.setPosition(OPEN);
        }
        else if (green>0.2 && green>red && green>blue && max<2.25) {
            telemetry.addData("Color","Green");
            greenGate.setPosition(OPEN);
        }
    }
    */

    /*
    void sort(double red, double green, double blue){
        double max;
        switch (sortState) {
            case IDLE:
                max = red+green+blue;
                if (max>0.3 && max<2.25){
                    sortState = SortState.SORTING;
                }
                break;
            case SORTING:
                max = red+green+blue;
                if (red > 0.2 && red>green && blue>green && max<2.25) {
                    telemetry.addData("Color","Purple");
                    SortPaddle.setPosition(leftSort);
                    sortTimer.reset();
                    sortState = SortState.WAIT;
                }
                else if (green>0.2 && green>red && green>blue && max<2.25) {
                    telemetry.addData("Color","Green");
                    SortPaddle.setPosition(GreenSort);
                    sortTimer.reset();
                    sortState = SortState.WAIT;
                }
                break;
            case WAIT:
                if (sortTimer.seconds() > SORT_TIME_SECONDS){
                    SortPaddle.setPosition(IdleSort);
                    sortState = SortState.IDLE;
                }
                break;
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
                    rightFeeder.setPower(MAX_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.SPIN_DOWN;
                }
                if (greenShotRequested){
                    leftFeeder.setPower(MAX_SPEED);
                    feederTimer.reset();
                    launchState = LaunchState.SPIN_DOWN;
                }
                break;
            case SPIN_DOWN:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launcher.setVelocity(STOP_SPEED);
                    leftFeeder.setPower(STOP_SPEED);
                    launchState = LaunchState.IDLE;
                }
                break;
        }
    }
}
*/