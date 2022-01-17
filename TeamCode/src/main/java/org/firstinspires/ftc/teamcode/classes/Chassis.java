package org.firstinspires.ftc.teamcode.classes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.classes.enums.TurnDirection;

public class Chassis {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    Gamepad gamepad1;
    double scaleFactor, leftPower, rightPower;

    LinearOpMode opMode;
    int headingOffset;

    final double driveSpeed = 0.6;
    final int ticsPerInch = 38;

    //Teleop constructor (no LinearOpMode reference)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;
        motorFrontLeft = hardwareMap.dcMotor.get("TLM10");
        motorBackLeft = hardwareMap.dcMotor.get("BLM11");
        motorFrontRight = hardwareMap.dcMotor.get("TRM12");
        motorBackRight = hardwareMap.dcMotor.get("BRM13");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Autonomous constructor (saves LinearOpMode then calls first constructor)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1, LinearOpMode opMode, int headingOffset){
        this(hardwareMap, gpad1);
        this.opMode = opMode;
    }

    public void actuate(){
        scaleFactor = driveSpeed + gamepad1.left_trigger * (1-driveSpeed);
        leftPower = -gamepad1.left_stick_y * scaleFactor;
        rightPower = -gamepad1.right_stick_y * scaleFactor;
        motorFrontLeft.setPower(leftPower);
        motorBackLeft.setPower(leftPower);
        motorFrontRight.setPower(rightPower);
        motorBackRight.setPower(rightPower);
    }

    public void stopDrive() {
        motorFrontRight.setPower(0);
        motorBackRight.setPower(0);
        motorFrontLeft.setPower(0);
        motorBackLeft.setPower(0);
    }

    public void forward_inches(double inches, double power){
        int target = (int) (inches * ticsPerInch) + motorFrontLeft.getCurrentPosition();
        int tolerance = 5;
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
        while(motorFrontLeft.getCurrentPosition() <= target-tolerance && this.opMode.opModeIsActive()){
        }
        this.stopDrive();
    }

    public void backward_inches(double inches, double power){
        int target = motorFrontLeft.getCurrentPosition()- (int) (inches * ticsPerInch);
        int tolerance = 5;
        motorBackLeft.setPower(-power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorFrontRight.setPower(-power);
        while(motorFrontLeft.getCurrentPosition() >= target-tolerance && this.opMode.opModeIsActive()){
        }
        this.stopDrive();
    }

    public void turnToHeading(double targetHeading, TurnDirection dir, double turnPower, double tolerance) {
        double turnScaling = 0.1; //TODO: coefficient to modify the error by, trial and error here
//        double imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).thirdAngle;
        double imuHeading = 0.0; //TODO: Get from encoders
        imuHeading = (imuHeading + 360) % 360;
        double curHeading = (imuHeading + headingOffset) % 360; // this is our 'adjusted' current heading
        int multiplier = dir == TurnDirection.RIGHT ? 1 : -1; // multiply powers by 1 if RIGHT, -1 if LEFT to get the turn direction
        double error = Math.abs(curHeading - targetHeading);
        motorFrontRight.setPower(-multiplier * turnPower);
        motorBackRight.setPower(-multiplier * turnPower);
        motorFrontLeft.setPower(multiplier * turnPower);
        motorBackLeft.setPower(multiplier * turnPower);
        while (error > tolerance && this.opMode.opModeIsActive()) {
//            imuHeading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle; //TODO: Get from encoders
            imuHeading = (imuHeading + 360) % 360;
            curHeading = (imuHeading + headingOffset) % 360;
            error = Math.abs(curHeading - targetHeading);
        }
        stopDrive();
    }

    public void driveSeconds(double power, double time) {
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
        this.opMode.sleep((long) (time * 1000));
        stopDrive();
    }
}
