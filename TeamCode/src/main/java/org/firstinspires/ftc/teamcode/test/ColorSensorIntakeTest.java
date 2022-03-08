package org.firstinspires.ftc.teamcode.test;

import android.animation.RectEvaluator;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;


@TeleOp
public class ColorSensorIntakeTest extends OpMode {
    MOEBot robot;
    ColorSensor color;
    //DistanceSensor distanceSensor;



    //int normalDistance = 2;

    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
        color = hardwareMap.get(ColorSensor.class, "dispenserColor");
        //distanceSensor = hardwareMap.get(DistanceSensor.class, "distanceSensor");
    }

    @Override
    public void loop() {
        isBlockIn();
        telemetry.addData("hello pls work", 1);
        telemetry.addData("red", color.red());
        telemetry.addData("green", color.green());
        telemetry.addData("blue", color.blue());
        telemetry.addData("isBlockIn", isBlockIn());
        //telemetry.addData("Distance:", distanceSensor.getDistance(DistanceUnit.INCH));
    }

    public boolean isBlockIn(){
        if((color.red() + color.blue() + color.green()) > 100){
            return true;
        } else return false;
    }

//    public boolean isBlockInDistance() {
//        if(distanceSensor.getDistance(DistanceUnit.INCH) < normalDistance) {
//            return true;
//        } else return false;
//    }



}
