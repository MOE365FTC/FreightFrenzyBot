package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.*;
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
        final double startTime = System.nanoTime();
        //TODO: coding at the pool
        //control drive motors
        robot.chassis.actuate();

        //control slides
        robot.slides.setExtension();
        robot.slides.setTilt();
        robot.slides.actuate();

        //control intake
        robot.intake.actuateIntake();

        //control dispenser
        robot.dispenser.setPivot();
        robot.dispenser.actuate();

        //control TSE system
        robot.tseArm.setArm();
        robot.tseArm.actuate();

        //control carousel
        robot.carousel.actuate();

        //imu
        robot.imu.getAngles();

        //telemetry
//        robot.slides.composeTelemetry(telemetry);
//        robot.dispenser.composeTelemetry(telemetry);
//        robot.imu.composeTelemetry(telemetry);
        robot.tseArm.composeTelemetry(telemetry);
        telemetry.update();
        telemetry.addData("Loop ms", (System.nanoTime() - startTime) / 1000000.0);
        //TODO: coding at the school
        //TODO: watch video -> https://www.youtube.com/watch?v=dQw4w9WgXcQ
    }
}
//code to survive