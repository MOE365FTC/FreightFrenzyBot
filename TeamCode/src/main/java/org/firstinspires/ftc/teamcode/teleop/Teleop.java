package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;

import org.firstinspires.ftc.teamcode.classes.*;
/*

    Mrs. Myers approves this code

*/

@TeleOp
public class Teleop extends OpMode {
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
        //control drive motors
        robot.chassis.actuate();

        //control slides
        robot.slides.setExtension();
        robot.slides.actuate();

        //control intake
        robot.intake.actuateIntake();

        //control dispenser
        robot.dispenser.actuate();

        //control carousel
        robot.carousel.actuate();

        //telemetry
        robot.slides.composeTelemetry(telemetry);
        robot.dispenser.composeTelemetry(telemetry);
        telemetry.update();
        //TODO: coding at the school
    }
}
//code to survive
/*
Coding by Jonas Ho



 */