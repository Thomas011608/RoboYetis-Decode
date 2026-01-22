package org.firstinspires.ftc.teamcode;
import androidx.annotation.NonNull;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.road_runner.MecanumDrive;
import com.acmerobotics.roadrunner.ParallelAction;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import java.util.Set;

@Config
@Autonomous(name = "RoadRunnerAutonomous", group = "Competition")
public class RoadRunnerAutonomous extends LinearOpMode {
    public class Launcher {
        CompetitionAutonomous Functions = new CompetitionAutonomous();

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
                double power = Functions.getPower();
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
        Pose2d initialPose = new Pose2d(11.8, 61.7, Math.toRadians(90));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);

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

        // actions that need to happen on init; for instance, a claw tightening.
        //Actions.runBlocking(claw.closeClaw());


        while (!isStopRequested() && !opModeIsActive()) {
            int position = visionOutputPosition;
            telemetry.addData("Position during Init", position);
            telemetry.update();
        }

        int startPosition = visionOutputPosition;
        telemetry.addData("Starting Position", startPosition);
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        Action trajectoryActionChosen;
        if (startPosition == 1) {
            trajectoryActionChosen = tab1.build();
        } else if (startPosition == 2) {
            trajectoryActionChosen = tab2.build();
        } else {
            trajectoryActionChosen = tab3.build();
        }

        Actions.runBlocking(
                new SequentialAction(
                        trajectoryActionChosen,
                        launcher.LaunchLeft(),
                        launcher.LaunchRight(),
                        trajectoryActionCloseOut
                )
        );
    }
}