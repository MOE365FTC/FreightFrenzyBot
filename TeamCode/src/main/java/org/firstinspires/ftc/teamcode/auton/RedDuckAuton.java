package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.enums.TSEPos;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;
import org.firstinspires.ftc.teamcode.hardware.MOEBot;

//NO DUCKS YET
@Config
@Autonomous
public class RedDuckAuton extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 90;
    public static int f1Top = 4; //fwd to grab
    public static int f1Mid = 4;
    public static int f1Bot = 4;
    public static int f2Top = 6; //fwd to drop preload
    public static int f2Mid = 4;
    public static int f2Bot = 2;
    public static int f3 = 22; //back toward ducks
    public static int f4 = 12; //go into floor goal
    public static int t1Top = 70; //turn to grab
    public static int t1Mid = 90;
    public static int t1Bot = 110;
    public static int t2 = 50; //turn to goal
    public static int t3 = 45; //turn to ducks
    public static int t4 = 120; //turn toward floor goal
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);

        waitForStart();

        TSEPos curCase = robot.TSETracker.getPosition();
        telemetry.addData("xpos", robot.TSETracker.getXPos());
        telemetry.addData("case", curCase);
        telemetry.update();

        robot.tseArm.autonDeploy();
        robot.chassis.forward_inches(4, 0.3);
        switch(curCase){
            case TOP:
                robot.chassis.turn(t1Top, 2);
                robot.chassis.forward_inches(f1Top, 0.3);
                robot.tseArm.autonGrab();
                robot.chassis.backward_inches(f1Top, 0.3); //cancel out prev forward so we're in the same spot for each case
                break;
            case MID:
                robot.chassis.turn(t1Mid, 2);
                robot.chassis.forward_inches(f1Mid, 0.3);
                robot.tseArm.autonGrab();
                robot.chassis.backward_inches(f1Mid, 0.3);
                break;
            case BOT:
                robot.chassis.turn(t1Bot, 2);
                robot.chassis.forward_inches(f1Bot, 0.3);
                robot.tseArm.autonGrab();
                robot.chassis.backward_inches(f1Bot, 0.3);
                break;
        }
        robot.chassis.turn(t2, 2); //turn to goal
        switch(curCase) {
            case TOP:
                robot.chassis.forward_inches(f2Top, 0.5);
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
                robot.chassis.forward_inches(f2Mid, 0.5);
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
                robot.chassis.forward_inches(f2Bot, 0.5);
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
        while(robot.slides.isBusy() && opModeIsActive()){
        }
        robot.slides.retractAndWait(); //ensure we are retracted and turn off motor
        robot.chassis.turn(t3, 2); //turn towards ducks
        robot.chassis.backward_inches(f3, 0.5); //go toward ducks
        robot.carousel.startRed();
        sleep(3000);
        robot.carousel.stop();
        robot.chassis.turn(t4, 2);
        robot.chassis.forward_inches(f4, 0.5);
        robot.chassis.setOdometryDown(false);
    }
}

