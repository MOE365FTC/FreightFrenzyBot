package org.firstinspires.ftc.teamcode.teleop;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
//@Config
public class MOEOpMode extends OpMode {
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    public DcMotorEx slideExtend;
    public DcMotorEx slideRotate;
    public DcMotorEx intake;
    public CRServo carousel;
    public Servo odoLift;
    public Servo outtake;

    public static int STORED_EXTEND = 0;
    public static int LOW_EXTEND = 250;
    public static int MID_EXTEND = 250;
    public static int HIGH_EXTEND = 250;

    public static int STORED_ROTATE = 0;
    public static int LOW_ROTATE = 100;
    public static int MID_ROTATE = 100;
    public static int HIGH_ROTATE = 100;

    public static int SLIDE_MULTIPLIER = 30;
    public static double INTAKE_POWER = 1.0;
    public static double INTAKE_CURRENT = 700;
    public static boolean holding = false;
    public static double OUTTAKE_OPEN = 1.0;
    public static double OUTTAKE_CLOSED = 0.0;
    public static double CAROUSEL_POWER = 1.0;
    ElapsedTime timer = new ElapsedTime();

    HardwareMap hwMap;

    public void init(HardwareMap aHwMap) {
        hwMap = aHwMap;
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
        outtake = hardwareMap.get(Servo.class, "OTS");
    }

    public void init() {

    }

    public void loop() {

    }
    public void wait(double waitTime) {
            timer.reset();
            while (timer.time() < waitTime) {
            }
    }
}
