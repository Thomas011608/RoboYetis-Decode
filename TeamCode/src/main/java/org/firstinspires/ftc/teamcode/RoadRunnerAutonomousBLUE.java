package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.road_runner.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "RoadRunnerAutonomousBLUE", group = "Competition")
public class RoadRunnerAutonomousBLUE extends LinearOpMode {
    //HEADER: Define Variables
    int ID = 0;
    double distance = -1;
    double power = -1;
    double X = -1;
    double GOAL_ANGLE_RAD = Math.PI + 0.44;

    //Define final variables
    final double STOP_SPEED = 0.0;
    final double MAX_SPEED = 1.0;
    final double FEED_TIME_SECONDS = 1.5;
    final double INTAKE_TIME_SECONDS = 0.3;
    final double INTAKE_IN_TIME_SECONDS = 5.0;

    //Define timers
    ElapsedTime rightFeederTimer = new ElapsedTime();
    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime backTimer = new ElapsedTime();

    //HEADER: Camera class
    public class Camera {
        //Initialize Camera
        private HuskyLens huskyLens;
        public Camera(HardwareMap hardwareMap) {
            huskyLens = hardwareMap.get(HuskyLens.class, "camera");
            if (!huskyLens.knock()) {
                telemetry.addData("HuskyLens", "Error initializing HuskyLens");
            }
        }

        //Set the Obelisk ID if it has not been set
        public class GetObeliskID implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 1 || block.id == 2 || block.id == 3) {
                        ID = block.id;
                    }
                }
                return ID == 0;
            }
        }
        public Action GetObeliskID() {
            return new GetObeliskID();
        }

        //Set the distance and power variables when the AprilTag detected has ID 4
        public class GetPowerRed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 4) {
                        //Custom distance function
                        double area = block.width * block.height;
                        distance = Math.pow((area / 16139259.8), (1 / -1.89076));
                    } else {
                        distance = -1;
                    }
                }

                //Custom power function
                if (distance < 130 && distance != -1) {
                    power = 1300;
                } else if (distance > 240) {
                    power = 1475;
                } else if (distance != -1){
                    power = 0.0360562 * (Math.pow(distance, 2)) - 11.25698 * distance + 2092.27902;
                } else {
                    power = -1;
                }
                return (power == -1);
            }
        }
        public Action GetPowerRed() {
            return new GetPowerRed();
        }

        //Set the distance and power variables when the AprilTag detected has ID 5
        public class GetPowerBlue implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 5) {
                        //Custom distance function
                        double area = block.width * block.height;
                        distance = Math.pow((area / 16139259.8), (1 / -1.89076));
                    } else {
                        distance = -1;
                    }
                }

                //Custom power function
                if (distance < 130 && distance != -1) {
                    power = 1300;
                } else if (distance > 240) {
                    power = 1475;
                } else if (distance != -1){
                    power = 0.0360562 * (Math.pow(distance, 2)) - 11.25698 * distance + 2092.27902;
                } else {
                    power = -1;
                }
                return power == -1;
            }
        }
        public Action GetPowerBlue() {
            return new GetPowerBlue();
        }

        /*
        //Set the X position variable when the AprilTag detected has ID 4
        public class GetTagXRed implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 4) {
                        X = block.x;
                    } else {
                        X = -1;
                    }
                }
                return X == -1;
            }
        }
        public Action GetTagXRed() {
            return new GetTagXRed();
        }

        //Set the X position variable when the AprilTag detected has ID 5
        public class GetTagXBlue implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 5) {
                        X = block.x;
                    } else {
                        X = -1;
                    }
                }
                return X == -1;
            }
        }
        public Action GetTagXBlue() {
            return new GetTagXBlue();
        }
        */
    }

    public class Launcher {
        private DcMotorEx launcher;
        private DcMotor feedRight;
        private DcMotor feedLeft;
        private DcMotor intake;

        public Launcher(HardwareMap hardwareMap) {
            launcher = hardwareMap.get(DcMotorEx.class, "launcher");
            launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));
            launcher.setDirection(DcMotorSimple.Direction.REVERSE);

            feedRight = hardwareMap.get(DcMotor.class, "right_feeder");
            feedRight.setDirection(DcMotorSimple.Direction.REVERSE);
            feedRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            feedLeft = hardwareMap.get(DcMotor.class, "left_feeder");
            feedLeft.setDirection(DcMotorSimple.Direction.FORWARD);
            feedLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            intake = hardwareMap.get(DcMotor.class, "intake");
            intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intake.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class SpinUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setVelocity(1350);
                return launcher.getVelocity() == 0;
            }
        }
        public Action SpinUp() {
            return new SpinUp();
        }

        public class SetTargetVelocity implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                power = 1450;
                double minPower = power - 50;
                double maxPower = power + 50;

                launcher.setVelocity(power);

                return !(launcher.getVelocity() <= maxPower) || !(launcher.getVelocity() >= minPower);
            }
        }

        public Action SetTargetVelocity() {
            return new SetTargetVelocity();
        }

        public class SpinDown implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setVelocity(STOP_SPEED);
                return false;
            }
        }

        public Action SpinDown() {
            return new SpinDown();
        }

        public class Intake implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(MAX_SPEED);
                    intakeTimer.reset();
                    initialized = true;
                }

                if (intakeTimer.seconds() > INTAKE_TIME_SECONDS) {
                    intake.setPower(STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action Intake() {
            return new Intake();
        }

        public class PickUp implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(MAX_SPEED);
                    intakeTimer.reset();
                    initialized = true;
                }

                if (intakeTimer.seconds() > INTAKE_IN_TIME_SECONDS) {
                    intake.setPower(STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action PickUp() {
            return new PickUp();
        }

        public class FeedBack implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    feedLeft.setPower(-MAX_SPEED);
                    feedRight.setPower(-MAX_SPEED);
                    backTimer.reset();
                }

                if (backTimer.seconds() > 0.25) {
                    feedLeft.setPower(STOP_SPEED);
                    feedRight.setPower(STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }

        public Action FeedBack() {
            return new FeedBack();
        }

        public class LaunchLeft implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    feedLeft.setPower(MAX_SPEED);
                    leftFeederTimer.reset();
                    initialized = true;
                }

                if (leftFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    feedLeft.setPower(STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action LaunchLeft(){
            return new LaunchLeft();
        }

        public class LaunchRight implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    feedRight.setPower(MAX_SPEED);
                    rightFeederTimer.reset();
                    initialized = true;
                }

                if (rightFeederTimer.seconds() > FEED_TIME_SECONDS) {
                    feedRight.setPower(STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action LaunchRight(){
            return new LaunchRight();
        }
    }

    @Override
    public void runOpMode() {
        Pose2d currentPose = new Pose2d(63, -12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);


        leftFeederTimer.reset();
        rightFeederTimer.reset();
        backTimer.reset();
        intakeTimer.reset();

        //Create Trajectories to build later
        TrajectoryActionBuilder goalAlign = drive.actionBuilder(currentPose)
                .lineToX(58)
                .turnTo(GOAL_ANGLE_RAD);
        currentPose = new Pose2d(58, -12, GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveToIntake = drive.actionBuilder(currentPose)
                .turnTo(Math.PI)
                .splineTo(new Vector2d(31, -36),3*Math.PI/2);
        currentPose = new Pose2d(31, -36, 3*Math.PI/2);

        TrajectoryActionBuilder driveWhileIntake = drive.actionBuilder(currentPose)
                .lineToY(-54, new TranslationalVelConstraint(5));
        currentPose = new Pose2d(39, -54, 3*Math.PI/2);

        TrajectoryActionBuilder moveToLaunch = drive.actionBuilder(currentPose)
                .splineToConstantHeading(new Vector2d(58, -12), 3*Math.PI/2)
                .turnTo(GOAL_ANGLE_RAD);
        currentPose = new Pose2d(58, -12, GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveForward = drive.actionBuilder(currentPose)
                .turnTo(Math.PI)
                .lineToX(24);


        while (!isStopRequested() && !opModeIsActive()) {
            Actions.runBlocking(camera.GetObeliskID());
            telemetry.addData("ID", ID);
            telemetry.update();
        }

        waitForStart();
        if (isStopRequested()) return;


        //HEADER: GPP
        if (ID == 1){
            Actions.runBlocking(
                    new ParallelAction(
                            //Turn and spin up
                            goalAlign.build(),
                            launcher.SpinUp(),

                            new SequentialAction(
                                    //Set spin velocity, launch in the correct order
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),

                                    //Green
                                    launcher.LaunchLeft(),

                                    //Purple 1
                                    launcher.LaunchRight(),

                                    //Purple 2
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.SpinDown(),

                                    //Move toward the intake & Collect balls
                                    driveToIntake.build(),
                                    new ParallelAction (
                                            launcher.PickUp(),
                                            driveWhileIntake.build()
                                    ),

                                    //Move back and spin up
                                    new ParallelAction(
                                            moveToLaunch.build(),
                                            launcher.SpinUp(),
                                            launcher.FeedBack()
                                    ),

                                    //Set the velocity and launch
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchLeft(),
                                    launcher.LaunchRight(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    //Move out of launch zone
                                    driveForward.build()
                            )
                    )
            );
        }

        //HEADER: PGP
        else if (ID == 2){
            Actions.runBlocking(
                    new ParallelAction(
                            //Turn and spin up
                            goalAlign.build(),
                            launcher.SpinUp(),

                            new SequentialAction(
                                    //Set spin velocity, launch in the correct order
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),

                                    //Purple 1
                                    launcher.LaunchRight(),

                                    //Green
                                    launcher.LaunchLeft(),

                                    //Purple 2
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.SpinDown(),

                                    //Move toward the intake & Collect balls
                                    driveToIntake.build(),
                                    new ParallelAction (
                                            launcher.PickUp(),
                                            driveWhileIntake.build()
                                    ),

                                    //Move back and spin up
                                    new ParallelAction(
                                            moveToLaunch.build(),
                                            launcher.SpinUp()
                                    ),

                                    //Set the velocity and launch
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    //Move out of launch zone
                                    driveForward.build()
                            )
                    )
            );
        }

        //HEADER: PPG
        else {
            Actions.runBlocking(
                    new ParallelAction(
                            //Turn and spin up
                            goalAlign.build(),
                            launcher.SpinUp(),

                            new SequentialAction(
                                    //Set spin velocity, launch in the correct order
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),

                                    //Purple 1
                                    launcher.LaunchRight(),

                                    //Purple 2
                                    launcher.Intake(),
                                    launcher.LaunchRight(),

                                    //Green
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    //Move toward the intake & Collect balls
                                    driveToIntake.build(),
                                    new ParallelAction (
                                            launcher.PickUp(),
                                            driveWhileIntake.build()
                                    ),

                                    //Move back and spin up
                                    new ParallelAction(
                                            moveToLaunch.build(),
                                            launcher.SpinUp()
                                    ),

                                    //Set the velocity and launch
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchRight(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    //Move out of launch zone
                                    driveForward.build()
                            )
                    )
            );
        }

    }
}