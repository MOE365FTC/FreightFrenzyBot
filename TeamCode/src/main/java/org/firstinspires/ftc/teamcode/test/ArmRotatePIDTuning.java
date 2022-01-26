package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.signedness.qual.Constant;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Config
@TeleOp
public class ArmRotatePIDTuning extends OpMode {
    MOEBot robot;
    int pos = 1180;
    public static double p = 20.0, i = 0.0, d = 0.0, f = 0.0;
        //    double p = 15.0, i = 2.0, d = 3.0, f = 0.0;
//    double p = 10.0, i = 2.0, d = 0.0, f = 5.0;
//    double p = 10.0, i = 2.0, d = 2.0, f = 12.0;
    boolean oldRB;
    boolean oldLB;
    @Override
    public void init() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2);

        robot.slides.updateState();
        telemetry.addData("SLIDES ARE", robot.slides.curSlideState);
    }

    @Override
    public void loop() {
        pos += -gamepad2.right_stick_y;

        if(gamepad2.right_bumper && !oldRB){
            p += 0.05;
            oldRB = true;
        } else if(gamepad2.left_bumper && !oldLB && p >= 0.1){
            p -= 0.05;
            oldLB = true;
        }

        if(oldRB && !gamepad2.right_bumper){
            oldRB = false;
        }
        if(oldLB && !gamepad2.left_bumper){
            oldLB = false;
        }
        if(gamepad2.y) {
            robot.slides.rotateRunToPosition(pos, 0.6, p, i, d, f);
        }
        robot.slides.composeTelemetry(telemetry);
        telemetry.addData("p", p);
        telemetry.addData("pos", pos);
    }
}
