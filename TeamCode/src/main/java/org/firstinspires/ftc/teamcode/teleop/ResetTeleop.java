package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
/*

    Mrs. Myers approves this code

*/

@TeleOp
public class ResetTeleop extends OpMode {
    MOEBot robot;

    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);
    }

    public void init_loop(){
        robot.slides.updateState();
        telemetry.addData("SLIDES ARE", robot.slides.curSlideState);
    }

    @Override
    public void loop() {
        //TODO: coding at the pool

        //control slides
        robot.slides.setExtension();
        robot.slides.setTilt();
        robot.slides.actuate();

        //control TSE system
        robot.tseArm.resetControl();

        //telemetry
//        robot.slides.composeTelemetry(telemetry);
//        robot.dispenser.composeTelemetry(telemetry);
//        robot.imu.composeTelemetry(telemetry);
        robot.tseArm.composeTelemetry(telemetry);
        telemetry.update();

        //TODO: coding at the school
    }
}
//code to survive
/*
Coding by Jonas Ho



 */