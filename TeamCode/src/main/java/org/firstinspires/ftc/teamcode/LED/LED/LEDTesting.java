package org.firstinspires.ftc.teamcode.LED.LED;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class LEDTesting extends OpMode {
    public Servo light = null;
    public Double x = 23.0;

    @Override
    public void init() {
        telemetry.addLine("Hello World");
        light = hardwareMap.get(Servo.class,"lightOne");
        light.setPosition( 0.279);
    }
    @Override
    public void loop() {
        x = Double.valueOf(gamepad1.left_stick_y);
        light.setPosition(-x);
        telemetry.addData("lightOne Value",x);

    }
}

