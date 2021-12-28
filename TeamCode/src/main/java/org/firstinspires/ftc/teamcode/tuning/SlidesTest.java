package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.teleop.MOEOpMode;
import org.firstinspires.ftc.teamcode.util.Toggle;


//@Config
@TeleOp
public class SlidesTest extends MOEOpMode {
    public static int EXTENSION = 100, ROTATE = 100;
//    MOEOpMode robot = new MOEOpMode();
    DcMotorEx slideExtend, slideRotate;
    public void init(){
//        robot.init(hardwareMap);
        slideExtend = hardwareMap.get(DcMotorEx.class, "SEM");
        slideRotate = hardwareMap.get(DcMotorEx.class, "SRM");
    }
    public void loop(){
        slideExtend.setTargetPosition((int) (gamepad1.right_stick_y * EXTENSION));
        slideRotate.setTargetPosition((int) (gamepad1.right_stick_x * ROTATE));

        if(gamepad1.x){
            slideExtend.setPower(1.0);
        } else if (gamepad1.y){
            slideExtend.setPower(-1.0);
        } else{
            slideExtend.setPower(0.0);
        }

        telemetry.addData("extensionTarget", slideExtend.getTargetPosition());
        telemetry.addData("extensionPosition", slideExtend.getCurrentPosition());
    }
}