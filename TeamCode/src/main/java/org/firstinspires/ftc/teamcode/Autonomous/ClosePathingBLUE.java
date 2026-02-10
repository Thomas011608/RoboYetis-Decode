package org.firstinspires.ftc.teamcode.Autonomous;
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
import org.firstinspires.ftc.teamcode.Autonomous.road_runner.MecanumDrive;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@Autonomous(name = "ClosePathingBLUE", group = "Test")
public class ClosePathingBLUE extends LinearOpMode {
    //HEADER: Define Variables
    int ID = 0;
    double distance = -1;
    double power = -1;
    double GOAL_ANGLE_RAD = Math.toRadians(-142);
    double spikeNumber = 2;

    //Define final variables
    final double STOP_SPEED = 0.0;
    final double MAX_SPEED = 1.0;
    final double FEED_TIME_SECONDS = 0.3;
    final double LAUNCH_INTAKE_TIME_SECONDS = 0.8;
    final double INTAKE_IN_TIME_SECONDS = 2.5;
    final double INTAKE_SPEED_ONE = 20;
    final double INTAKE_SPEED_TWO = 20;
    final double LAUNCH_POWER = 1225;


    public static class coordinatesClose{
        public double launchX = -24;
        public double launchY = -24;
        public double driveToIntakeX = -11;
        public double driveToIntakeY = -28;
        public double driveWhileIntakeX = -14;
        public double driveWhileIntakeY = -56;
        public double moveToLaunchX = -24;
        public double moveToLaunchY = -24;
        public double driveToIntake2X = 15;
        public double driveToIntake2Y = -30;
        public double driveWhileIntake2X = 10;
        public double driveWhileIntake2Y = -68;
        public double reverseY = -48;
        public double reverse2Y = -48;
    }
    public static coordinatesClose MAP = new coordinatesClose();


    //Define timers
    ElapsedTime rightFeederTimer = new ElapsedTime();
    ElapsedTime leftFeederTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime backTimer = new ElapsedTime();
    ElapsedTime waitTimer = new ElapsedTime();

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
                    if ((block.id == 1 || block.id == 2 || block.id == 3) && ID == 0) {
                        ID = block.id;
                    }
                }
                return false;
            }
        }

        public Action GetObeliskID() {
            return new GetObeliskID();
        }
    }

    //HEADER: Launcher Class
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
            intake.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        //HEADER: Wait Class
        public class Wait implements Action {
            boolean initialized = false;
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized){
                    waitTimer.reset();
                    initialized = true;
                }
                return !(waitTimer.seconds() > 0.5);
            }
        }
        public Action Wait(){
            return new Wait();
        }

        //HEADER: SpinUp Class
        public class SpinUp implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                launcher.setVelocity(1350);
                return launcher.getVelocity() == 0;
            }
        }
        public Action SpinUp() {
            return new SpinUp();
        }

        //HEADER: SetTargetVelocity Class
        public class SetTargetVelocity implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                power = LAUNCH_POWER;
                double minPower = power - 50;
                double maxPower = power + 50;

                launcher.setVelocity(power);

                return !(launcher.getVelocity() <= maxPower) || !(launcher.getVelocity() >= minPower);
            }
        }
        public Action SetTargetVelocity() {
            return new SetTargetVelocity();
        }


        //HEADER: SpinDown Class
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

        //HEADER: Intake Class
        public class Intake implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intake.setPower(MAX_SPEED);
                    intakeTimer.reset();
                    initialized = true;
                }

                if (intakeTimer.seconds() > LAUNCH_INTAKE_TIME_SECONDS) {
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

        //HEADER: PickUp Class
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

        //FeedBack Class
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

        //HEADER: LaunchLeft Class
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

        //HEADER: LaunchRight Class
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
        //HEADER: Initialize OpMode
        Pose2d currentPose = new Pose2d(-54, -54, Math.toRadians(-135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        leftFeederTimer.reset();
        rightFeederTimer.reset();
        backTimer.reset();
        intakeTimer.reset();

        /*
        //HEADER: Create Trajectories to build later
        TrajectoryActionBuilder goalAlign = drive.actionBuilder(currentPose)
                .lineToX(-24, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(-24, 24, Math.toRadians(135));

        TrajectoryActionBuilder turnToExit = drive.actionBuilder(currentPose)
                .turnTo(GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveToIntake = drive.actionBuilder(currentPose)
                .turnTo(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(-14, 40), Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(-14, 40, Math.PI / 2);

        TrajectoryActionBuilder driveWhileIntake = drive.actionBuilder(currentPose)
                .lineToY(66, new TranslationalVelConstraint(INTAKE_SPEED_ONE));
        currentPose = new Pose2d(-14, 66, Math.PI / 2);

        TrajectoryActionBuilder reverseToLaunch = drive.actionBuilder(currentPose)
                .lineToY(48)
                .splineToLinearHeading(new Pose2d(-24, 24, GOAL_ANGLE_RAD), Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(-24, 24, GOAL_ANGLE_RAD);
        TrajectoryActionBuilder driveToIntakeTwo = drive.actionBuilder(currentPose)
                .turnTo(Math.PI / 2)
                .splineToConstantHeading(new Vector2d(10, 40), Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(10, 36, Math.PI / 2);

        TrajectoryActionBuilder driveWhileIntakeTwo = drive.actionBuilder(currentPose)
                .lineToY(74, new TranslationalVelConstraint(INTAKE_SPEED_TWO));
        currentPose = new Pose2d(10, 74, Math.PI / 2);

        TrajectoryActionBuilder reverseToLaunchTwo = drive.actionBuilder(currentPose)
                .lineToY(48)
                .splineToLinearHeading(new Pose2d(-24, 24, GOAL_ANGLE_RAD), Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(-24, 24, GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveAway = drive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(45))
                .lineToX(0, new TranslationalVelConstraint(80));
         */

        TrajectoryActionBuilder goalAlign = drive.actionBuilder(currentPose)
                .lineToX(MAP.launchX, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(MAP.launchX, MAP.launchY, Math.toRadians(-135));


        TrajectoryActionBuilder driveToIntake = drive.actionBuilder(currentPose)
                .turnTo(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(MAP.driveToIntakeX, MAP.driveToIntakeY), -Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(MAP.driveToIntakeX, MAP.driveToIntakeY, -Math.PI / 2);

        TrajectoryActionBuilder driveWhileIntake = drive.actionBuilder(currentPose)
                .lineToY(MAP.driveWhileIntakeY, new TranslationalVelConstraint(INTAKE_SPEED_ONE));
        currentPose = new Pose2d(MAP.driveWhileIntakeX, MAP.driveWhileIntakeY, -Math.PI / 2);

        TrajectoryActionBuilder reverseToLaunch = drive.actionBuilder(currentPose)
                .lineToY(MAP.reverseY)
                .splineToLinearHeading(new Pose2d(MAP.moveToLaunchX, MAP.moveToLaunchY, GOAL_ANGLE_RAD), -Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(MAP.moveToLaunchX, MAP.moveToLaunchY, GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveToIntakeTwo = drive.actionBuilder(currentPose)
                .turnTo(-Math.PI / 2)
                .splineToConstantHeading(new Vector2d(MAP.driveToIntake2X, MAP.driveToIntake2Y), -Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(MAP.driveToIntake2X, MAP.driveToIntake2Y, -Math.PI / 2);

        TrajectoryActionBuilder driveWhileIntakeTwo = drive.actionBuilder(currentPose)
                .lineToY(MAP.driveWhileIntake2Y, new TranslationalVelConstraint(INTAKE_SPEED_TWO));
        currentPose = new Pose2d(MAP.driveWhileIntake2X, MAP.driveWhileIntake2Y, -Math.PI / 2);

        TrajectoryActionBuilder reverseToLaunchTwo = drive.actionBuilder(currentPose)
                .lineToY(MAP.reverse2Y)
                .splineToLinearHeading(new Pose2d(MAP.moveToLaunchX, MAP.moveToLaunchY, GOAL_ANGLE_RAD), -Math.PI / 2, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(MAP.moveToLaunchX, MAP.moveToLaunchY, GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveAway = drive.actionBuilder(currentPose)
                .turnTo(Math.toRadians(-45))
                .lineToX(-12, new TranslationalVelConstraint(80));


        waitForStart();
        if (isStopRequested()) return;

        //HEADER: Run Pathing
        Actions.runBlocking(
                new SequentialAction(
                        //Launch 1
                        goalAlign.build(),
                        launcher.Wait(),

                        //Launch 2
                        driveToIntake.build(),
                        launcher.Wait(),
                        driveWhileIntake.build(),
                        launcher.Wait(),
                        reverseToLaunch.build(),
                        launcher.Wait(),

                        //Launch 3
                        driveToIntakeTwo.build(),
                        launcher.Wait(),
                        driveWhileIntakeTwo.build(),
                        launcher.Wait(),
                        reverseToLaunchTwo.build(),

                        launcher.Wait(),
                        driveAway.build()
                )
        );
    }
}