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
@Autonomous(name = "CloseAutonomousRED", group = "Competition")
public class CloseAutonomousRED extends LinearOpMode {
    //HEADER: Define Variables
    int ID = 0;
    double distance = -1;
    double power = -1;
    double GOAL_ANGLE_RAD = Math.toRadians(142);

    //Define final variables
    final double STOP_SPEED = 0.0;
    final double MAX_SPEED = 1.0;
    final double FEED_TIME_SECONDS = 0.3;
    final double LAUNCH_INTAKE_TIME_SECONDS = 0.8;
    final double INTAKE_IN_TIME_SECONDS = 2.5;
    final double INTAKE_SPEED_ONE = 20;
    final double INTAKE_SPEED_TWO = 20;
    final double LAUNCH_POWER = 1225;

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
        Pose2d currentPose = new Pose2d(-54, 54, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        leftFeederTimer.reset();
        rightFeederTimer.reset();
        backTimer.reset();
        intakeTimer.reset();

        while (!isStopRequested() && !opModeIsActive()) {
            Actions.runBlocking(camera.GetObeliskID());
            telemetry.addData("ID", ID);
            telemetry.update();
        }
        //HEADER: Create Trajectories to build later
        TrajectoryActionBuilder goalAlign = drive.actionBuilder(currentPose)
                .lineToX(-24, new TranslationalVelConstraint(80));
        currentPose = new Pose2d(-24, 24, Math.toRadians(135));

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

        waitForStart();
        if (isStopRequested()) return;

        Actions.runBlocking(
                new ParallelAction(
                        goalAlign.build(),
                        new SequentialAction(
                                launcher.SpinUp(),

                                launcher.SetTargetVelocity(),
                                launcher.LaunchLeft(),
                                launcher.LaunchRight(),
                                launcher.Intake(),
                                launcher.LaunchRight(),

                                driveToIntake.build(),

                                new ParallelAction(
                                        launcher.PickUp(),
                                        driveWhileIntake.build()
                                ),

                                reverseToLaunch.build(),

                                launcher.SetTargetVelocity(),
                                launcher.LaunchLeft(),
                                launcher.LaunchRight(),
                                launcher.Intake(),
                                launcher.LaunchRight(),
                                launcher.LaunchLeft(),

                                driveToIntakeTwo.build(),

                                new ParallelAction(
                                        launcher.PickUp(),
                                        driveWhileIntakeTwo.build()
                                ),

                                reverseToLaunchTwo.build(),

                                launcher.SetTargetVelocity(),
                                launcher.LaunchLeft(),
                                launcher.LaunchRight(),
                                launcher.Intake(),
                                launcher.LaunchRight(),
                                launcher.LaunchLeft(),

                                driveAway.build()
                        )
                )
        );
    }
}