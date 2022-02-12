package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.enums.TSEPos;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;

@Autonomous
public class RedWarehouseAuton extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 90;
    public void runOpMode() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);

        waitForStart();
        TSEPos curCase = robot.TSETracker.getPosition();
//        curCase = TSEPos.TOP;
        telemetry.addData("xpos", robot.TSETracker.getXPos());
        telemetry.addData("case", curCase);
        telemetry.update();

        //Move away from wall and face alliance hub
        robot.chassis.forward_inches(16, 0.6);
        robot.chassis.turnToHeading(135, TurnDirection.LEFT, 0.8, 2);

        switch(curCase) {
            case TOP:
                robot.chassis.forward_inches(2  , 0.6);
                robot.slides.autonArm(1800, 1580);
                sleep(1000);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.144);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(3, 0.3);
                break;
            case MID:
                robot.chassis.forward_inches(1.8, 0.3);
                robot.slides.autonArm(2000,900);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.2);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(2.5, 0.5);
                break;
            case BOT:
//                robot.chassis.backward_inches(0.5, 0.5);
                robot.slides.autonArm(2400,780);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.22);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(2.5, 0.5);
                break;
            default:
                break;
        }
        robot.slides.updateState();
        robot.dispenser.setTilt(0.0);
        robot.slides.autonArm((int) robot.slides.rotateTicsDeltaToVertical + 500, 700);

        robot.chassis.turnToHeading(180.0, TurnDirection.LEFT, 0.8, 2);
        robot.chassis.setOdometryDown(false);
        while(robot.slides.isBusy() && opModeIsActive()){
        }
        telemetry.addData("ROT", robot.slides.getCurrentRotation());
        telemetry.update();
        robot.slides.retractAndWait(); //ensure we are retracted and turn off motor
        sleep(500);
        robot.chassis.driveSeconds(-0.8, 1.2);
    }
}
