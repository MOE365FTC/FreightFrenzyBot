package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.TurnDirection;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;

@Disabled
@Autonomous
public class OdometryTest extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 270;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);

        waitForStart();
        robot.chassis.forward_inches(24, 0.3);
        robot.chassis.backward_inches(24, 0.3);
//        robot.chassis.forward_inches(24, 0.5);
        //robot.chassis.turnToHeading(180, TurnDirection.RIGHT, 0.5, 2);
        while(opModeIsActive()) {
            robot.chassis.composeTelemetry(telemetry);
            telemetry.update();
        }
    }
}
