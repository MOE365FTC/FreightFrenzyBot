package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class FreightDetectionTest extends OpMode {
    MOEBot robot;
    ColorSensor color;

    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
        color = hardwareMap.get(ColorSensor.class, "color");
    }

    @Override
    public void loop() {
        telemetry.addData("red", color.red());
        telemetry.addData("green", color.green());
        telemetry.addData("blue", color.blue());
    }

}