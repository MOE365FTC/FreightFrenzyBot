package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.enums.TSEPos;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;

@Config
@Autonomous
public class BlueWarehouseAuton extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 270;
    public static int heading1 = 225;
    public static double forward1 = 6;
    public static int heading2 = 315;
    public static double forward2 = 7;
    public static double backward1 = 1;
    public static int heading3 = 225;
    public static double forward3 = 6;
    public void runOpMode() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);

        waitForStart();
        TSEPos curCase = robot.TSETracker.getPosition();
//        curCase = TSEPos.TOP;
        telemetry.addData("xpos", robot.TSETracker.getXPos());
        telemetry.addData("case", curCase);
        telemetry.update();

        robot.tseArm.autonDeploy();

        //Move away from wall
        robot.chassis.forward_inches(6, 0.4);

        switch(curCase) {
            case TOP:
                //TSE GRABBING
                robot.chassis.turn(253, 1);
                while(opModeIsActive() && robot.tseArm.isBusy()){} // wait for arm to finish lowering
                robot.chassis.forward_inches(10, 0.5); // move towards TSE
                robot.tseArm.autonGrab();
                sleep(1000);
                robot.tseArm.autonLift();// pick up TSE

                //PRELOAD SCORING
                robot.chassis.turn(230, 1);
                robot.chassis.forward_inches(10, 0.6); // move towards hub
                robot.slides.autonArm(1800, 1580); // raise arm
                sleep(1000);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.144);
                sleep(500);
                robot.dispenser.setGateOpen(true); // score
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(6, 0.3); // back away
                break;
            case MID:
                //TSE GRABBING
                robot.chassis.turn(275, 1); // face TSE
                while(opModeIsActive() && robot.tseArm.isBusy()){} // wait for arm to finish lowering
                robot.chassis.forward_inches(8, 0.5); // move towards TSE
                robot.tseArm.autonGrab();
                sleep(1000);
                robot.tseArm.autonLift();// grab
                // PRELOAD SCORING
                robot.chassis.turn(230, 1); // face hub
                robot.chassis.forward_inches(9, 0.3);
                robot.slides.autonArm(2400,900);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.22);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(5.5, 0.5);
                break;
            case BOT:
                //TSE GRABBING
                robot.chassis.turn(225, 1); // face away from barricade
                while(opModeIsActive() && robot.tseArm.isBusy()){} // wait for arm to finish lowering
                robot.chassis.forward_inches(6, 0.5); // move away from barricade
                robot.chassis.turn(320, 1); // face TSE
                robot.chassis.forward_inches(7, 0.5); // move towards TSE
                robot.tseArm.autonGrab();
                sleep(1000);
                robot.tseArm.autonLift();// grab TSE
                //PRELOAD SCORING
                robot.chassis.backward_inches(1, 0.5); // back away from barricade
                robot.chassis.turn(225, 1); // face hub
                robot.chassis.forward_inches(6, 0.5); // move towards hub
                robot.slides.autonArm(2400,780);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.22);
                sleep(500);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(3, 0.5);
                break;
            default:
                break;
        }
        robot.slides.updateState();
        robot.dispenser.setTilt(0.0);
        robot.slides.autonArm((int) robot.slides.rotateTicsDeltaToVertical + 500, 700);

        robot.chassis.turn(180, 1);
        robot.chassis.setOdometryDown(false);
        while(robot.slides.isBusy() && opModeIsActive()){
        }
        telemetry.addData("ROT", robot.slides.getCurrentRotation());
        telemetry.update();
        robot.slides.retractAndWait(); //ensure we are retracted and turn off motor
        sleep(500);
        robot.chassis.driveSeconds(-0.8, 1.5);
    }
}
