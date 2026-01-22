package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
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

@Config
@Autonomous(name = "RoadRunnerAutonomous", group = "Competition")
public class RoadRunnerAutonomous extends LinearOpMode {
    int ID = 0;
    double distance = -1;
    double power = -1;
    double X = -1;
    final double GOAL_ANGLE_RAD = Math.PI - 0.48995732625;
    public class Camera {
        private HuskyLens huskyLens;
        public Camera(HardwareMap hardwareMap) {
            huskyLens = hardwareMap.get(HuskyLens.class, "camera");
            if (!huskyLens.knock()) {
                telemetry.addData("HuskyLens", "Error initializing HuskyLens");
            }
        }

        public class GetObeliskID implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (ID == 0 && block.id == 1 || block.id == 2 || block.id == 3) {
                        ID = block.id;
                    }
                }
                return ID == 0;
            }
        }
        public Action GetObeliskID() {
            return new GetObeliskID();
        }

        public class GetPowerRed implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 4) {
                        //custom distance function
                        double area = block.width * block.height;
                        distance = Math.pow((area / 16139259.8), (1 / -1.89076));
                    } else {
                        distance = -1;
                    }
                }

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

        public class GetPowerBlue implements Action {
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                HuskyLens.Block[] blocks = huskyLens.blocks();
                for (HuskyLens.Block block : blocks) {
                    if (block.id == 5) {
                        //custom distance function
                        double area = block.width * block.height;
                        distance = Math.pow((area / 16139259.8), (1 / -1.89076));
                    } else {
                        distance = -1;
                    }
                }

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
    }

    CompetitionAutonomous Functions = new CompetitionAutonomous();
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
                launcher.setVelocity(Functions.STOP_SPEED);
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
                    intake.setPower(Functions.MAX_SPEED);
                    Functions.intakeTimer.reset();
                }

                if (Functions.intakeTimer.seconds() > Functions.INTAKE_TIME_SECONDS) {
                    intake.setPower(Functions.STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action Intake() {
            return new Intake();
        }

        public class LaunchLeft implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    feedLeft.setPower(Functions.MAX_SPEED);
                    Functions.feederTimer.reset();
                }

                if (Functions.feederTimer.seconds() > Functions.FEED_TIME_SECONDS) {
                    feedLeft.setPower(Functions.STOP_SPEED);
                    return false;
                } else {
                    return true;
                }
            }
        }
        public Action LaunchLeft() {
            return new LaunchLeft();
        }

        public class LaunchRight implements Action {
            boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    feedRight.setPower(Functions.MAX_SPEED);
                    Functions.feederTimer.reset();
                }

                if (Functions.feederTimer.seconds() > Functions.FEED_TIME_SECONDS) {
                    feedRight.setPower(Functions.STOP_SPEED);
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
        Pose2d currentPose = new Pose2d(63, 12, Math.toRadians(180));
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);
        Launcher launcher = new Launcher(hardwareMap);
        Camera camera = new Camera(hardwareMap);

        // vision here that outputs position
        int visionOutputPosition = 1;

        /*
        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .setTangent(Math.toRadians(0))
                .lineToX(32)
                .strafeTo(new Vector2d(44.5, 30))
                .turn(Math.toRadians(180))
                .lineToX(47.5)
                .waitSeconds(3);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .lineToY(37)
                .setTangent(Math.toRadians(0))
                .lineToX(18)
                .waitSeconds(3)
                .setTangent(Math.toRadians(0))
                .lineToXSplineHeading(46, Math.toRadians(180))
                .waitSeconds(3);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(180))
                .waitSeconds(2)
                .strafeTo(new Vector2d(46, 30))
                .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.endTrajectory().fresh()
                .strafeTo(new Vector2d(48, 12))
                .build();
         */


        TrajectoryActionBuilder goalAlign = drive.actionBuilder(currentPose)
                .lineToX(58)
                .turnTo(GOAL_ANGLE_RAD);
        currentPose = new Pose2d(58,12,GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveToIntake = drive.actionBuilder(currentPose)
                .turnTo(Math.PI)
                .splineTo(new Vector2d(39,36),Math.PI/2);
        currentPose = new Pose2d(39,36,Math.PI/2);

        TrajectoryActionBuilder driveWhileIntake = drive.actionBuilder(currentPose)
                .lineToY(54);
        currentPose = new Pose2d(39,54,Math.PI/2);

        TrajectoryActionBuilder moveToLaunch = drive.actionBuilder(currentPose)
                .turnTo(GOAL_ANGLE_RAD)
                .splineToConstantHeading(new Vector2d(58,12), GOAL_ANGLE_RAD);
        currentPose = new Pose2d(58,12,GOAL_ANGLE_RAD);

        TrajectoryActionBuilder driveForward = drive.actionBuilder(currentPose)
                .turnTo(Math.PI)
                .lineToX(24);


        //TODO: Add alliance color selection

        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            Actions.runBlocking(camera.GetObeliskID());
            telemetry.addData("ID", ID);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        /*if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }*/

        //HEADER: GPP
        if (ID == 1){
            Actions.runBlocking(
                    new ParallelAction(
                            goalAlign.build(),
                            launcher.SpinUp(),
                            new SequentialAction(
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchLeft(),
                                    launcher.LaunchRight(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    driveToIntake.build(),
                                    new ParallelAction (
                                            launcher.Intake(),
                                            driveWhileIntake.build()
                                    ),
                                    new ParallelAction(
                                            moveToLaunch.build(),
                                            launcher.SpinUp()
                                    ),

                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchLeft(),
                                    launcher.LaunchRight(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    driveForward.build()
                            )
                    )
            );
        }

        //HEADER: PGP
        else if (ID == 2){
            Actions.runBlocking(
                    new ParallelAction(
                            goalAlign.build(),
                            launcher.SpinUp(),
                            new SequentialAction(
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    driveToIntake.build(),
                                    new ParallelAction (
                                            launcher.Intake(),
                                            driveWhileIntake.build()
                                    ),
                                    new ParallelAction(
                                            moveToLaunch.build(),
                                            launcher.SpinUp()
                                    ),

                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.Intake(),
                                    launcher.LaunchRight(),
                                    launcher.LaunchLeft(),
                                    launcher.SpinDown(),

                                    driveForward.build()
                            )
                    )
            );
        }

        //HEADER: PPG
        else {
            Actions.runBlocking(
                    new ParallelAction(
                            goalAlign.build(),
                            launcher.SpinUp(),
                            new SequentialAction(
                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchLeft(),
                                    launcher.Intake(),
                                    launcher.LaunchLeft(),
                                    launcher.LaunchRight(),
                                    launcher.SpinDown(),

                                    driveToIntake.build(),
                                    new ParallelAction (
                                            launcher.Intake(),
                                            driveWhileIntake.build()
                                    ),
                                    new ParallelAction(
                                            moveToLaunch.build(),
                                            launcher.SpinUp()
                                    ),

                                    camera.GetPowerRed(),
                                    launcher.SetTargetVelocity(),
                                    launcher.LaunchLeft(),
                                    launcher.Intake(),
                                    launcher.LaunchLeft(),
                                    launcher.LaunchRight(),
                                    launcher.SpinDown(),

                                    driveForward.build()
                            )
                    )
            );
        }

    }
}