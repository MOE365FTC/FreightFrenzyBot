package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MOEOpMode extends OpMode {
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    DcMotorEx slideExtend;
    DcMotorEx slideRotate;

    DcMotorEx intake;
    CRServo carousel;
    Servo odoLift;

    int EXTENSION = 250;
    int ROTATE = 100;
    int SLIDE_MULTIPLIER = 30;

    ElapsedTime timer = new ElapsedTime();
    @Override
    public void init() {
        frontLeft = hardwareMap.get(DcMotor.class, "FLM");
        frontRight = hardwareMap.get(DcMotor.class, "FRM");
        backLeft = hardwareMap.get(DcMotor.class, "BLM");
        backRight = hardwareMap.get(DcMotor.class, "BRM");

        //TODO: Reverse Motors

        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM");
        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM");

        slideExtend.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideExtend.setTargetPosition(0);
        slideExtend.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        slideRotate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideRotate.setTargetPosition(0);
        slideRotate.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        intake = hardwareMap.get(DcMotorEx.class, "INM");
        carousel = hardwareMap.get(CRServo.class, "CSS");
        odoLift = hardwareMap.get(Servo.class, "OLS");
    }

    @Override
    public void loop() {

    }
    public void wait(double waitTime) {
            timer.reset();
            while (timer.time() < waitTime) {
            }
    }
}
