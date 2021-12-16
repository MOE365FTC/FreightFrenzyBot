package org.firstinspires.ftc.teamcode.tuning;

//import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.teleop.MOEOpMode;

@TeleOp
//@Config
public class Carousel extends MOEOpMode {
    MOEOpMode robot = new MOEOpMode();
    public static double POWER = 0.5;
    public void init(){
        robot.init(hardwareMap);
    }

    public void loop(){
        if(gamepad1.x){
            carousel.setPower(POWER);
        }
    }
}
