package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@TeleOp
public class BarrierCrossingTest extends OpMode {

    MOEBot robot;
    double maxX, maxY, maxZ;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
    }

    @Override
    public void loop() {
        robot.chassis.actuate();
        robot.imu.getAngles();
        robot.imu.composeTelemetry(telemetry);
        maxX = Math.max(maxX, robot.imu.x);
        maxY = Math.max(maxY, robot.imu.y);
        maxZ = Math.max(maxZ, robot.imu.z);
        telemetry.addData("max X", maxX);
        telemetry.addData("max Y", maxY);
        telemetry.addData("max Z", maxZ);
    }
}
