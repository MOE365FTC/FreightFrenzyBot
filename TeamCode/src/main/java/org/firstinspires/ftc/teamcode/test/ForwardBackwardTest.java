package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Config
@Autonomous
public class ForwardBackwardTest extends LinearOpMode {
    MOEBot robot;
    DcMotor left, right;
    public static int targetPos = 24;
    public static boolean forwards = false;
    public static boolean runFront = false;
    public static boolean runBack = false;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, 0);
//        if(runBack) {
//            robot.chassis.leftDrive.back.setPower(1);
//            robot.chassis.rightDrive.back.setPower(1);
//        }
//        if(runFront) {
//            robot.chassis.leftDrive.front.setPower(1);
//            robot.chassis.leftDrive.front.setPower(1);
//        }
//        while(opModeIsActive()){}
        waitForStart();
        if(forwards){
            robot.chassis.forward_inches(targetPos, 0.5);
        } else{
            robot.chassis.backward_inches(targetPos, 0.5);
        }
        while(opModeIsActive()){
            robot.chassis.composeTelemetry(telemetry);
            telemetry.update();
        }
    }
}