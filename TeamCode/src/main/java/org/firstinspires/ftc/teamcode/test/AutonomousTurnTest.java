package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.checkerframework.checker.units.qual.Angle;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class AutonomousTurnTest extends LinearOpMode {

    BNO055IMU imu;
    double HEADING_OFFSET = 90; // starting heading of the robot, change this for every auton starting pose
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;

    enum Direction{
        LEFT,
        RIGHT
    }

    @Override
    public void runOpMode(){
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode to make this JSON
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new BNO055IMU.Parameters().accelerationIntegrationAlgorithm;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        telemetry.addData("STATUS", "IMU Initializing...");
        telemetry.update();
        imu.initialize(parameters);
        telemetry.addData("STATUS", "Ready!");
        telemetry.update();

        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        //NOTE: the FTC example code starts the IMU integration. This integrates the acceleration into velocity and position.
        // However, only poor implementations of this are included, so we may as well not use it. If we want to try it,
        // we would have to write our own integrator schema that would reject noise.
        // To just get the heading (what we want) we don't have to integrate the accelerometer; that is read off the Gyro.

        turnToHeading(0, Direction.RIGHT, 0.3, 3); // left turn to face heading 0 (towards back wall)

    }

    public void turnToHeading(double targetHeading, Direction dir, double turnPower, double tolerance){
        //targetHeading: final heading to face
        //dir: Direction to turn
        //minPower: minimum power to turn the robot at
        //maxPower: max power to turn the robot at (if min == max, constant turning speed)
        //tolerance: tolerance (in degrees) to stop the turn at
        double turnScaling = 0.1; //TODO: coefficient to modify the error by, trial and error here
        double imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        imuHeading = (imuHeading + 360) % 360;
        double curHeading = (imuHeading + HEADING_OFFSET) % 360; // this is our 'adjusted' current heading
        int multiplier = dir == Direction.RIGHT ? 1 : -1; // multiply powers by 1 if RIGHT, -1 if LEFT to get the turn direction
        double error = Math.abs(curHeading - targetHeading);
        motorFrontRight.setPower(-multiplier * turnPower);
        motorBackRight.setPower(-multiplier * turnPower);
        motorFrontLeft.setPower(multiplier * turnPower);
        motorBackLeft.setPower(multiplier * turnPower);
        while(error > tolerance && opModeIsActive()){ // ALWAYS put an opModeIsActive() check in all FTC while loops!!!
            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            imuHeading = (imuHeading + 360) % 360;
            curHeading = (imuHeading + HEADING_OFFSET) % 360;
            error = Math.abs(curHeading - targetHeading);
//            turnPower = clamp(error * turnScaling, minPower, maxPower);

//            sleep(10); // sleep for 10 milliseconds, we can tune this number too.
            telemetry.addData("error", error);
            telemetry.update();
        }
        stopDrive();
        while(opModeIsActive()){
            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            imuHeading = (imuHeading + 360) % 360;
            curHeading = (imuHeading + HEADING_OFFSET) % 360;
            telemetry.addData("imuHeading", imuHeading);
            telemetry.addData("curHeading", curHeading);
            error = Math.abs(curHeading - targetHeading);
            telemetry.addData("error", error);
            telemetry.update();
        }

    }

    public void stopDrive(){
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }

    public double clamp(double value, double min, double max){
        if(value < min)
            return min;
        else if(value > max)
            return max;
        else
            return value;
    }
}
