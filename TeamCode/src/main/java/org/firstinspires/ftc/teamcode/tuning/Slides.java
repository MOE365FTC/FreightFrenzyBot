package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.MOEOpMode;
import org.firstinspires.ftc.teamcode.Toggle;

@Config
@TeleOp
public class Slides extends MOEOpMode {
    public static int EXTENSION = 100;
    public static int ROTATE = 100;
    MOEOpMode robot = new MOEOpMode();
    public void init(){
        robot.init(hardwareMap);
    }
    public void loop(){
        slideExtend.setTargetPosition((int) (gamepad1.right_stick_y * EXTENSION));
        slideRotate.setTargetPosition((int) (gamepad1.right_stick_x * EXTENSION));

        if(Toggle.xToggled){
            slideExtend.setPower(1.0);
        } else {
            slideExtend.setPower(0.0);
        }
        if(Toggle.yToggled){
            slideRotate.setPower(1.0);
        } else {
            slideRotate.setPower(0.0);
        }

        telemetry.addData("extensionTarget", slideExtend.getTargetPosition());
        telemetry.addData("extensionPosition", slideExtend.getCurrentPosition());
    }
}