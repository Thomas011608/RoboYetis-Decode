package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.Arrays;
import java.util.concurrent.TimeUnit;
import android.graphics.Color;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "DistanceLauncherSpeed")
public class DistanceLauncherSpeed extends LinearOpMode {

    private final int READ_PERIOD = 1;
    private static HuskyLens huskyLens = null;
    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;
    double LAUNCH_MOTOR_SET_SPEED = 0;
    double LAUNCH_MOTOR_MIN_SPEED = LAUNCH_MOTOR_SET_SPEED - 100;
    double LAUNCH_TIME_SECONDS = 1;
    double MAX_SPEED = 1;
    double STOP_SPEED = 0;
    public double[] MotorSpeed;
    public double[] Distance;
    ElapsedTime launchTimer = new ElapsedTime();


    @Override
    public void runOpMode()
    {
        // HEADER: Configure HuskyLens
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        if (!huskyLens.knock()) {
            telemetry.addData(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            telemetry.addData(">>", "Press start to continue");
        }
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        // HEADER: Configure Launcher Motor
        launcher = hardwareMap.get(DcMotorEx.class,"launcher");
        //Set Launcher Direction
        launcher.setDirection(DcMotor.Direction.REVERSE);
        launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 5, 10, 25));
        //Set Launcher Zero Power Behavior
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //HEADER: Configure Feeder Servos
        leftFeeder = hardwareMap.get(CRServo.class,"left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class,"right_feeder");
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);
        //Set Servo Direction
        leftFeeder.setDirection(CRServo.Direction.FORWARD);
        rightFeeder.setDirection(CRServo.Direction.REVERSE);

        // HEADER: Wait for start
        telemetry.update();
        waitForStart();
        launchTimer.reset();

        while(opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue;
            }
            rateLimit.reset();

            HuskyLens.Block[] blocks = huskyLens.blocks();
            telemetry.addData("Block count", blocks.length);
            for (int i = 0; i < blocks.length; i++) {
                telemetry.addData("Block", blocks[i].toString());

                double area = blocks[i].width*blocks[i].height;
                double distance = Math.pow((area/16139259.8),(1/-1.89076));
                telemetry.addData("Distance", distance);
                telemetry.addData("Area", area);
            }

            if (gamepad1.aWasPressed()) {
                LAUNCH_MOTOR_SET_SPEED += 100;
                LAUNCH_MOTOR_MIN_SPEED = LAUNCH_MOTOR_SET_SPEED - 100;
            }

            if (gamepad1.bWasPressed()) {
                LAUNCH_MOTOR_SET_SPEED -= 100;
                LAUNCH_MOTOR_MIN_SPEED = LAUNCH_MOTOR_SET_SPEED - 100;
            }

            if (gamepad1.xWasPressed()) {
                launcher.setVelocity(LAUNCH_MOTOR_SET_SPEED);
                launchTimer.reset();
            }

            if (gamepad1.yWasPressed()) {
                launchTimer.reset();
                leftFeeder.setPower(MAX_SPEED);
                rightFeeder.setPower(MAX_SPEED);
            }

            if (launchTimer.seconds() >= LAUNCH_TIME_SECONDS + 9) {
                launcher.setVelocity(STOP_SPEED);
                launcher.setPower(STOP_SPEED);
                leftFeeder.setPower(STOP_SPEED);
                rightFeeder.setPower(STOP_SPEED);
            }

            double distance = GetDistance();
            telemetry.addData("getDistance", distance);
            //double power = 0.117582*(Math.pow(distance,2)) - 37.19797*distance + 4136.25617;
            //double power = 0.0610355*(Math.pow(distance,2)) - 19.20547*distance + 2718.58097;
            double power;
            if (distance<130){
                power = 1300;
            } else if (distance>240){
                power = 1475;
            } else {
                power = 0.0360562*(Math.pow(distance,2)) - 11.25698*distance + 2092.27902;
            }
            telemetry.addData("Power",power);

            if (gamepad1.dpadUpWasPressed()){
                LAUNCH_MOTOR_SET_SPEED = power;
                LAUNCH_MOTOR_MIN_SPEED = LAUNCH_MOTOR_SET_SPEED - 100;
            }

            telemetry.addData("Target Speed", LAUNCH_MOTOR_SET_SPEED);
            telemetry.addData("Current Velocity", launcher.getVelocity());
            telemetry.update();
        }
    }
    public static double GetDistance(){
        double distance = 0;
        HuskyLens.Block[] blocks = huskyLens.blocks();
        for (int i = 0; i < blocks.length; i++) {
            double area = blocks[i].width*blocks[i].height;
            distance = Math.pow((area/16139259.8),(1/-1.89076));
        }
        return distance;
    }
}