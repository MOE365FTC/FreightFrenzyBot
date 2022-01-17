package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.MOEBot;
import org.firstinspires.ftc.teamcode.enums.TSEPos;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;


@Autonomous
public class BlueWarehouseAuton extends LinearOpMode {
    MOEBot robot;
    final int HEADING_OFFSET = 270;
    public void runOpMode() {
        robot = new MOEBot(hardwareMap, gamepad1, gamepad2, this, HEADING_OFFSET);
//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode to make this JSON
//        parameters.loggingEnabled      = false;
//        parameters.loggingTag          = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new BNO055IMU.Parameters().accelerationIntegrationAlgorithm;
//        imu = hardwareMap.get(BNO055IMU.class, "imu"); //TODO: Move imu
//        imu.initialize(parameters);
//        telemetry.addData("STATUS", "IMU Initializing...");
//        telemetry.update();
//        imu.initialize(parameters);
//        telemetry.addData("STATUS", "Ready!");
//        telemetry.update();

        waitForStart();
        TSEPos curCase = robot.TSETracker.getPosition();
        telemetry.addData("xpos", robot.TSETracker.getXPos());
        telemetry.addData("case", curCase);
        telemetry.update();

        //Move away from wall and face alliance hub
        robot.chassis.forward_inches(24, 0.5);
        robot.chassis.turnToHeading(210.0, TurnDirection.RIGHT, 0.3, 2);

        switch(curCase) {
            case TOP:
                robot.chassis.forward_inches(1, 0.3);
                robot.slides.autonArm(2650, 1580);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.144);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(3, 0.5);
                break;
            case MID:
                robot.chassis.forward_inches(2.5, 0.3);
                robot.slides.autonArm(2800,780);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.22);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(2.5, 0.5);
                break;
            case BOT:
                robot.chassis.backward_inches(1.0, 0.5);
                robot.slides.autonArm(2950,780);
                while(robot.slides.isBusy() && opModeIsActive()){
                }
                robot.dispenser.setTilt(0.24);
                robot.dispenser.setGateOpen(true);
                sleep(1000);
                robot.dispenser.setGateOpen(false);
                robot.chassis.backward_inches(1.5, 0.5);
                break;
            default:
                break;
        }
        robot.slides.updateState();
        robot.dispenser.setTilt(0.0);
        robot.slides.autonSlideTilt(0.5, robot.slides.rotateTicsDeltaToVertical, 10);
        robot.slides.retractAndWait();
        while(robot.slides.isBusy() && opModeIsActive()){

        }
        robot.chassis.turnToHeading(190.0, TurnDirection.RIGHT, 0.3, 2);
        robot.chassis.driveSeconds(-0.8, 1.1);
    }
}
