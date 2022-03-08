package org.firstinspires.ftc.teamcode.hardware;

import androidx.appcompat.widget.MenuItemHoverListener;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.enums.SlideState;

public class Intake {
    Gamepad gamepad2, gamepad1;
    DcMotorEx intake;
    Slides slides;
    Dispenser dispenser;
    final double freightCurrentThreshold = 3800;
    public Intake(HardwareMap hardwareMap, Gamepad gpad1, Gamepad gpad2, Slides slides, Dispenser dispenser){
        this.gamepad1 = gpad1;
        this.gamepad2 = gpad2;
        this.slides = slides;
        this.dispenser = dispenser;

        intake = hardwareMap.get(DcMotorEx.class, "INM12");
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public boolean hasFreight(){
        return this.intake.getCurrent(CurrentUnit.MILLIAMPS) > freightCurrentThreshold;
    }
    public void actuateIntake() {
        if(gamepad2.right_trigger > 0 && slides.curSlideState == SlideState.RETRACTED){
            intake.setPower(gamepad2.right_trigger);
            if(this.hasFreight()){
                gamepad1.rumble(500);
                gamepad2.rumble(500);
            }
        } else if(gamepad2.right_bumper){
            intake.setPower(-1.0);
        } else{
            intake.setPower(0.0);
        }
    }

    public void composeTelemetry(Telemetry telemetry){
        telemetry.addLine("--INTAKE--");
        telemetry.addData("current", intake.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
