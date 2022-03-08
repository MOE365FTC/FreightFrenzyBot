package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Config
@Autonomous
public class TurnTest extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 0;
    public static int targetHeading = 270;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);

        waitForStart();

        robot.chassis.turn(targetHeading, 0.1f);
        while(opModeIsActive()){
            telemetry.addData("heading", robot.chassis.getHeading());
            telemetry.update();
        };
    }
}
