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
        imu.initialize(parameters);

        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");
        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();
        //NOTE: the FTC example code starts the IMU integration. This integrates the acceleration into velocity and position.
        // However, only poor implementations of this are included, so we may as well not use it. If we want to try it,
        // we would have to write our own integrator schema that would reject noise.
        // To just get the heading (what we want) we don't have to integrate the accelerometer; that is read off the Gyro.

        turnToHeading(0, Direction.LEFT, 0.3, 0.4, 5); // left turn to face heading 0 (towards back wall)
        sleep(5000); //wait 5 seconds
        turnToHeading(90, Direction.RIGHT, 0.3, 0.4, 5); //right turn to face heading 90 (towards RED alliance)
        sleep(5000); // wait 5 seconds
        turnToHeading(180, Direction.LEFT, 0.3, 0.4, 5); // left turn to face heading 180 (towards audience), the 'long way'

    }

    public void turnToHeading(double targetHeading, Direction dir, double minPower, double maxPower, double tolerance){
        //targetHeading: final heading to face
        //dir: Direction to turn
        //minPower: minimum power to turn the robot at
        //maxPower: max power to turn the robot at (if min == max, constant turning speed)
        //tolerance: tolerance (in degrees) to stop the turn at
        double turnScaling = 0.1; //TODO: coefficient to modify the error by, trial and error here
        double imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        double curHeading = imuHeading + HEADING_OFFSET; // this is our 'adjusted' current heading
        int multiplier = dir == Direction.RIGHT ? 1 : -1; // multiply powers by 1 if RIGHT, -1 if LEFT to get the turn direction
        double error = Math.abs(curHeading - targetHeading);
        double turnPower = clamp(error * turnScaling, minPower, maxPower); // get a power to turn at, won't ever go below min or over max power
        while(error > tolerance && opModeIsActive()){ // ALWAYS put an opModeIsActive() check in all FTC while loops!!!
            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
            curHeading = imuHeading + HEADING_OFFSET;
            error = Math.abs(curHeading - targetHeading);
            turnPower = clamp(error * turnScaling, minPower, maxPower);
            motorFrontRight.setPower(multiplier * turnPower);
            motorBackRight.setPower(multiplier * turnPower);
            motorFrontLeft.setPower(-1 * multiplier * turnPower);
            motorBackLeft.setPower(-1 * multiplier * turnPower);
//            sleep(10); // sleep for 10 milliseconds, we can tune this number too.
            telemetry.addData("error", error);
            telemetry.addData("1", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
            telemetry.addData("2", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).secondAngle);
            telemetry.addData("3", imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle);
            telemetry.update();
        }

        stopDrive();

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
