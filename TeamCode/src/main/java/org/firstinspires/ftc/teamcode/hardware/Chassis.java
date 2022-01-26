package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.enums.TurnDirection;

public class Chassis {
    DcMotor motorFrontLeft, motorBackLeft, motorFrontRight, motorBackRight;
    Servo servoOdoLeft, servoOdoRight;
    Gamepad gamepad1;
    LinearOpMode opMode;
    double scaleFactor, leftPower, rightPower;

    // Set heading offset in autonomous
    int headingOffset;

    // drive variables
    final double driveSpeed = 0.6;
    double heading = headingOffset;
    final int ticsPerInch = 1450;
    final int turnTicsPerDegree = 310;
    final double turnSlowPower = 0.2;

    final double odometryLeftUpPos = 0.0;
    final double odometryLeftDownPos = 1.0;
    final double odometryRightUpPos = 1.0;
    final double odometryRightDownPos = 0.0;

    //Teleop constructor (no LinearOpMode reference)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1){
        this.gamepad1 = gpad1;
        motorFrontLeft =  hardwareMap.get(DcMotor.class, "TLM10");
        motorBackLeft =   hardwareMap.get(DcMotor.class, "BLM11");
        motorFrontRight = hardwareMap.get(DcMotor.class, "TRM12");
        motorBackRight =  hardwareMap.get(DcMotor.class, "BRM13");

        servoOdoRight = hardwareMap.get(Servo.class, "ORS24");
        servoOdoLeft = hardwareMap.get(Servo.class, "OLS25");

        motorFrontLeft.setDirection(DcMotor.Direction.REVERSE);
        motorBackRight.setDirection(DcMotor.Direction.REVERSE);
        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.setOdometryDown(false); // raise odometry
        this.zeroEncoders();
    }

    //Autonomous constructor (saves LinearOpMode then calls first constructor)
    public Chassis(HardwareMap hardwareMap, Gamepad gpad1, LinearOpMode opMode, int headingOffset){
        this(hardwareMap, gpad1);
        this.opMode = opMode;
        this.headingOffset = headingOffset;
        this.setOdometryDown(true); // lower odometry
        this.zeroEncoders();
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

    public void setOdometryDown(boolean down){
        if (down){
            servoOdoRight.setPosition(odometryRightDownPos);
            servoOdoLeft.setPosition(odometryLeftDownPos);
        } else {
            servoOdoRight.setPosition(odometryRightUpPos);
            servoOdoLeft.setPosition(odometryLeftUpPos);
        }
    }

    public void zeroEncoders(){
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double clampDegrees(double raw){
        // take a raw degree input and wrap it to be between 0 and 360
        while(raw > 360){
            raw -= 360;
        }
        while(raw < 0){
            raw += 360;
        }
        return raw;
    }

    public double getHeading(){
        // get our heading from the axial encoders
        // return in degrees
        double raw = (motorFrontRight.getCurrentPosition() - motorFrontLeft.getCurrentPosition()) / turnTicsPerDegree;
        return clampDegrees(raw + headingOffset);
    }

    public void forward_inches(double inches, double power){
        int target = (int) (inches * ticsPerInch) + motorFrontRight.getCurrentPosition();
        int tolerance = 5;
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
        while(motorFrontRight.getCurrentPosition() <= target-tolerance && this.opMode.opModeIsActive()){

        }
        this.stopDrive();
    }

    public void backward_inches(double inches, double power){
        int target = motorFrontRight.getCurrentPosition() - (int)(inches * ticsPerInch);
        int tolerance = 5;
        motorBackLeft.setPower(-power);
        motorFrontLeft.setPower(-power);
        motorBackRight.setPower(-power);
        motorFrontRight.setPower(-power);
        while(motorFrontRight.getCurrentPosition() >= target-tolerance && this.opMode.opModeIsActive()){
        }
        this.stopDrive();
    }

    public void driveSeconds(double power, double time) {
        motorBackLeft.setPower(power);
        motorFrontLeft.setPower(power);
        motorBackRight.setPower(power);
        motorFrontRight.setPower(power);
        this.opMode.sleep((long) (time * 1000));
        stopDrive();
    }

    public void setLeftPower(double power, double time){
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        this.opMode.sleep((long) time * 1000);
        stopDrive();
    }

    public void setRightPower(double power, double time){
        motorBackLeft.setPower(power);
        motorBackRight.setPower(power);
        this.opMode.sleep((long) time * 1000);
        stopDrive();
    }

    public void turnToHeading(double targetHeading, TurnDirection dir, double turnPower, double tolerance){
        double heading = this.getHeading();
        int multiplier = dir == TurnDirection.RIGHT ? 1 : -1;
        double error = Math.abs(heading - targetHeading);

        while (error > tolerance && this.opMode.opModeIsActive()){
            if (error < 30)
                turnPower = turnSlowPower;
            motorFrontRight.setPower(-multiplier * turnPower);
            motorBackRight.setPower(-multiplier * turnPower);
            motorFrontLeft.setPower(multiplier * turnPower);
            motorBackLeft.setPower(multiplier * turnPower);
            heading = this.getHeading();
            error = Math.abs(heading - targetHeading);
        }

        stopDrive();
    }

    @Deprecated
    public void turnToHeadingIMU(double targetHeading, TurnDirection dir, double turnPower, double tolerance) {
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

    public void composeTelemetry(Telemetry telemetry){
        double sum = motorFrontRight.getCurrentPosition() + motorFrontLeft.getCurrentPosition();
        telemetry.addData("left", motorFrontLeft.getCurrentPosition());
        telemetry.addData("right", motorFrontRight.getCurrentPosition());
        telemetry.addData("sum", sum);
        telemetry.addData("scaled", sum / this.turnTicsPerDegree);
        telemetry.addData("heading", this.getHeading());
    }

}
