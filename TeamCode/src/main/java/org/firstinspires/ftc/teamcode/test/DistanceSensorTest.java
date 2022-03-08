package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class DistanceSensorTest extends OpMode {
    MOEBot robot;
    Rev2mDistanceSensor distanceSensor;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
        distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "RDS02");
    }

    @Override
    public void loop() {
        telemetry.addData("distance", distanceSensor.getDistance(DistanceUnit.INCH));
    }
}
