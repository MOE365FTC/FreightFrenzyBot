package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Config
@Autonomous
public class ForwardBackwardTest extends LinearOpMode {
    MOEBot robot;
    public static int targetPos = 48;
    public static boolean forwards = true;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, 0);

        waitForStart();

        if(forwards){
            robot.chassis.forward_inches(targetPos, 0.5);
        } else{
            robot.chassis.backward_inches(targetPos, 0.5);
        }
        while(opModeIsActive()){
            robot.chassis.composeTelemetry(telemetry);
            telemetry.update();
        };
    }
}