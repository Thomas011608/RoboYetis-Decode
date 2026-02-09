package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Autonomous.road_runner.MecanumDrive;

@Config
@Autonomous(name = "RoadrunnerTest", group = "Test")
public class RoadrunnerTest extends LinearOpMode {
    public void runOpMode() {
        Pose2d currentPose = new Pose2d(0,0,0);
        MecanumDrive drive = new MecanumDrive(hardwareMap, currentPose);

        TrajectoryActionBuilder moveForward = drive.actionBuilder(currentPose)
                .lineToX(72);

        TrajectoryActionBuilder moveLeft = drive.actionBuilder(currentPose)
                .strafeTo(new Vector2d(0,72));

        TrajectoryActionBuilder moveRight = drive.actionBuilder(currentPose)
                .strafeTo(new Vector2d(0,-72));

        TrajectoryActionBuilder moveBack = drive.actionBuilder(currentPose)
                .lineToX(-72);

        TrajectoryActionBuilder square = drive.actionBuilder(currentPose)
                .lineToX(24)
                .turnTo(Math.PI/2)
                .lineToY(24)
                .turnTo(Math.PI)
                .lineToX(-24)
                .turnTo(3*Math.PI/2)
                .lineToY(-24)
                .turnTo(0)
                .lineToX(24)
                .turnTo(3*Math.PI/4)
                .lineToX(0);

        drive = new MecanumDrive(hardwareMap, new Pose2d(63, -12, Math.toRadians(180)));

        TrajectoryActionBuilder launchToZero = drive.actionBuilder(new Pose2d(63, -12, Math.toRadians(180)))
                .splineTo(new Vector2d(0,0), Math.toRadians(180));

        TrajectoryActionBuilder chosenAction = moveForward;
        String actionName = "Forward";

        while (!opModeIsActive() && !isStopRequested()) {
            if (gamepad1.dpadUpWasPressed()) {
                chosenAction = moveForward;
                actionName = "Forward";
            }
            if (gamepad1.dpadLeftWasPressed()) {
                chosenAction = moveLeft;
                actionName = "Left";
            }
            if (gamepad1.dpadRightWasPressed()) {
                chosenAction = moveRight;
                actionName = "Right";
            }
            if (gamepad1.dpadDownWasPressed()) {
                chosenAction = moveBack;
                actionName = "Back";
            }
            if (gamepad1.aWasPressed()) {
                chosenAction = launchToZero;
                actionName = "From launch to (0,0)";
            }
            if (gamepad1.yWasPressed()) {
                chosenAction = square;
                actionName = "Square";
            }
            telemetry.addData("Action", actionName);
            telemetry.update();
        }

        waitForStart();

        Actions.runBlocking(chosenAction.build());
    }
}
