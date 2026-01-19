package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Disabled
@Autonomous (name = "CompetitionAutonomous", group = "Competition")
public class CompetitionAutonomous extends LinearOpMode {
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
    private HuskyLens huskyLens; //huskyLens is the variable of our camera

    // HEADER: Defining timers
    ElapsedTime feederTimer = new ElapsedTime();

    // HEADER: Defining final variables
    final int POSITION_ALIGNMENT_PIXELS = 15; // The range (+- this amount) of pixels the tag can be when aligned with the goal.
    final double FEED_TIME_SECONDS = 1.0; //The feeder servos run this long when a shot is requested.
    final double MAX_SPEED = 1.0; //We send this power to the servos when we want them to stop.
    final double STOP_SPEED = 0.0;
    final double DRIVING_SPEED_MULTIPLIER = 0.6;
    final double TURN_TIME_SECONDS = 0.8;

    // HEADER: Define other variables
    private int GoalID = 0;
    private int obeliskID = 0;

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

        // HEADER: Feeder Motor Definitions
        leftFeeder = hardwareMap.get(DcMotor.class, "left_feeder");
        rightFeeder = hardwareMap.get(DcMotor.class, "right_feeder");
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        //Set Servo Direction
        leftFeeder.setDirection(DcMotor.Direction.FORWARD);
        rightFeeder.setDirection(DcMotor.Direction.REVERSE);

        // HEADER: Intake Motor Definitions
        intake = hardwareMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


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

        launcher.setVelocity(1525);

        while (launcher.getVelocity()<1475){

        }

        purpleLaunch();
        greenLaunch();
        intake.setPower(MAX_SPEED);
        purpleLaunch();
        intake.setPower(STOP_SPEED);

        sleep(1000);

        launcher.setVelocity(STOP_SPEED);

        moveForward(0.8);

        sleep(500);

        moveForward(STOP_SPEED);
        stop();

        /*//HEADER: Find Obelisk + Move forward

        // Move forward for 0.5 seconds
        moveForward(0.5*DRIVING_SPEED_MULTIPLIER);
        while (opModeIsActive() && runtime.seconds() < 0.5) {
            telemetry.clearAll();
            telemetry.addData("Active now", "Finding Obelisk, %4.1f Seconds", runtime.seconds());
            telemetry.update();
        }
        runtime.reset();

        //Move forward until the Obelisk is detected
        while (opModeIsActive()) {
            getDistance();
            if (obeliskID != 0 || runtime.seconds() > 5) {
                break;
            }
            telemetry.clearAll();
            telemetry.addData("Active now","Finding Obelisk, %4.1f Seconds", runtime.seconds());
            telemetry.update();
        }
        telemetry.update();
        moveForward(STOP_SPEED);
        double forwardTime = runtime.seconds();
        runtime.reset();

        //HEADER: Leg 2, Move back to the launch zone.
        moveForward(-0.5*DRIVING_SPEED_MULTIPLIER);
        while  (opModeIsActive() && runtime.seconds() < forwardTime) {
            telemetry.clearAll();
            telemetry.addData("Leg", "2, Moving backward, %4.1f Seconds", runtime.seconds());
            telemetry.update();
        }
        moveForward(STOP_SPEED);


        //HEADER: Leg 3, align with the goal
        telemetry.addData("Leg", "Aligning");
        telemetry.update();

        //X and Xprev are initialized to either the rightmost or leftmost part of the screen.
        //(for Red/Blue alliance respectively)
        double X;
        double Xprev;
        if (GoalID == 4) {
            X = 320;
        } else {
            X = 0;
        }
        Xprev = X;

        //Run a loop until the X coordinate is between the position alignment.
        while (opModeIsActive()) {
            telemetry.clearAll();
            telemetry.addData("X", X);
            telemetry.addData("Xprev", Xprev);
            telemetry.update();

            //If the X-coordinate is negative (X < 0), we know the tag was not detected,
            //so we set it to the previous reading of the tag.
            X = getXCoordinate();
            if (X < 0) {
                X = Xprev;
            }

            //Turn the robot to align the tag, and if aligned, break the loop.
            if (X < 160 - POSITION_ALIGNMENT_PIXELS) {
                turnLeft(0.3 * DRIVING_SPEED_MULTIPLIER);
            }
            if (X > 160 + POSITION_ALIGNMENT_PIXELS) {
                turnRight(0.3 * DRIVING_SPEED_MULTIPLIER);
            }
            if (X > 160 - POSITION_ALIGNMENT_PIXELS && X < 160 + POSITION_ALIGNMENT_PIXELS ) {
                break;
            }
            //Update the value of Xprev.
            Xprev = X;
        }
        moveForward(STOP_SPEED);

        //HEADER: Launch the balls
        //Set the power value & spinup the launcher.
        double power = getPower();
        double minPower = power - 100;
        telemetry.addData("Leg", "Launching at speed %4.1f", power);
        telemetry.update();

        launcher.setVelocity(power);

        while (opModeIsActive() && launcher.getVelocity() < minPower) {
            telemetry.clearAll();
            telemetry.addData("Power", power);
            telemetry.addData("Actual Speed", launcher.getVelocity());
            telemetry.update();
        }

        // Launch the balls in the order dictated by the obelisk. The intake needs to be started
        // before launching the second purple ball to ensure it is in the feeder wheels.
        if (opModeIsActive() && obeliskID == 1) {
            greenLaunch();
            purpleLaunch();

            intake.setPower(MAX_SPEED);

            purpleLaunch();
        }
        if (opModeIsActive() && obeliskID == 2) {
            purpleLaunch();
            greenLaunch();

            intake.setPower(MAX_SPEED);

            purpleLaunch();
        }
        if (opModeIsActive() && obeliskID == 3) {
            purpleLaunch();

            intake.setPower(MAX_SPEED);

            purpleLaunch();
            greenLaunch();
        }

        //Spin down the launcher
        intake.setPower(STOP_SPEED);
        launcher.setVelocity(STOP_SPEED);
        launcher.setPower(STOP_SPEED);

        stop();*/
    }

    // HEADER: moveForward() function
    public void moveForward(double power) {
        // Set Motor Powers
        frontLeftDrive.setPower(power);
        backLeftDrive.setPower(power);
        frontRightDrive.setPower(power);
        backRightDrive.setPower(power);
    }

    //HEADER: turnLeft() function
    public void turnLeft(double power) {
        //Set Motor Powers
        frontLeftDrive.setPower(-power);
        frontRightDrive.setPower(power);
        backLeftDrive.setPower(-power);
        backRightDrive.setPower(power);
    }

    //HEADER: turnRight() function
    public void turnRight(double power) {
        //Set Motor Powers
        frontLeftDrive.setPower(DRIVING_SPEED_MULTIPLIER*power);
        frontRightDrive.setPower(-DRIVING_SPEED_MULTIPLIER*power);
        backLeftDrive.setPower(DRIVING_SPEED_MULTIPLIER*power);
        backRightDrive.setPower(-DRIVING_SPEED_MULTIPLIER*power);
    }

    //HEADER: getDistance() function
    public double getDistance() {
        //Get the tags that the camera sees. For obelisk tags, set obeliskID, for goal tags, find distance.
        double distance = -1;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            if ((block.id == 1 || block.id == 2 || block.id == 3) && obeliskID == 0) {
                obeliskID = block.id;
            }
            if (obeliskID != 0 && block.id == GoalID) {
                //custom distance function
                double area = block.width * block.height;
                distance = Math.pow((area / 16139259.8), (1 / -1.89076));
            }
        }
        return distance;
    }

    //HEADER: getPower() function
    public double getPower() {
        // Only set the distance after a valid distance has been measured.
        double distance = getDistance();
        while (distance == -1) {
            distance = getDistance();
        }

        // Custom power function
        double power;
        if (distance < 130) {
            power = 1300;
        } else if (distance > 240) {
            power = 1475;
        } else {
            power = 0.0360562 * (Math.pow(distance, 2)) - 11.25698 * distance + 2092.27902;
        }
        return power;
    }

    //HEADER: getXCoordinate() function
    public double getXCoordinate() {
        // The X Coordinate is defaulted to -1, if a tag is detected, the X-coordinate is set.
        double XCoord = -1;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (HuskyLens.Block block : blocks) {
            if (block.id == GoalID) {
                XCoord = block.x;
            }
        }
        return XCoord;
    }

    //HEADER: greenLaunch() function
    public void greenLaunch() {
        // The green ball is at the right side.
        rightFeeder.setPower(MAX_SPEED);
        feederTimer.reset();

        while (feederTimer.seconds() < FEED_TIME_SECONDS) {
            telemetry.clearAll();
            telemetry.addData("Launch", "Green");
            telemetry.update();
        }

        rightFeeder.setPower(STOP_SPEED);
    }

    //HEADER: purpleLaunch() function
    public void purpleLaunch() {
        // The purple balls are on the left side.
        leftFeeder.setPower(MAX_SPEED);
        feederTimer.reset();

        while (feederTimer.seconds() < FEED_TIME_SECONDS) {
            telemetry.clearAll();
            telemetry.addData("Launch", "Purple");
            telemetry.update();
        }

        leftFeeder.setPower(STOP_SPEED);
        telemetry.update();
    }
}