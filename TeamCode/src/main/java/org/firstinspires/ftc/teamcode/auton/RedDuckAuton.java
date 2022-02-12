package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.TSEPos;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;

//NO DUCKS YET
@Autonomous
public class RedDuckAuton extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 90;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);

        waitForStart();

        TSEPos curCase = robot.TSETracker.getPosition();
        telemetry.addData("xpos", robot.TSETracker.getXPos());
        telemetry.addData("case", curCase);
        telemetry.update();

        robot.chassis.forward_inches(12, 0.3);
        robot.chassis.turnToHeading(60, TurnDirection.LEFT, 0.6, 4);
        switch(curCase) {
            case TOP:
                robot.slides.autonArm(1800, 1580);
                sleep(1000);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.144);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                break;
            case MID:
                robot.slides.autonArm(2000,900);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.2);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                break;
            case BOT:
                robot.slides.autonArm(2400,780);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.22);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                break;
            default:
                break;
        }
        robot.slides.updateState();
        robot.dispenser.setTilt(0.0);
        robot.slides.autonArm((int) robot.slides.rotateTicsDeltaToVertical + 500, 700);
        robot.chassis.setOdometryDown(false);
        while(robot.slides.isBusy() && opModeIsActive()){
        }
        robot.slides.retractAndWait(); //ensure we are retracted and turn off motor
    }
}

