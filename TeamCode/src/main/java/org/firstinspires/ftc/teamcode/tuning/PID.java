package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class PID {
    ElapsedTime timer = new ElapsedTime();
    public void VeloPIDTuner(DcMotorEx motor, double Target, double AcceptableError){
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;
        double f = 0.0;
        while(gamepad1.a) {
            double previousVelocity = 0.0;
            double previousTime = 0.0;
            double errorD = Math.abs((previousVelocity - motor.getVelocity()) / Math.abs(previousTime - timer.time()));

            motor.setVelocity(Target);

            double error = Math.abs(motor.getVelocity()-Target);
            motor.setVelocityPIDFCoefficients(p, i, d, f);
            if(error > AcceptableError){
                p+= 0.03;
            } else if(errorD > 5){
                d+= 0.03;
            }
            previousVelocity = motor.getVelocity();
            previousTime = timer.time();
        }
    }
    public void PositionPIDTuner(DcMotorEx motor, double Target, double AcceptableError){
        double p = 0.0;
        double i = 0.0;
        double d = 0.0;
        double f = 0.0;
        while(gamepad1.a) {
            double previousVelocity = 0.0;
            double previousTime = 0.0;
            double errorD = Math.abs((previousVelocity - motor.getVelocity()) / Math.abs(previousTime - timer.time()));

            motor.setVelocity(Target);

            double error = Math.abs(motor.getVelocity()-Target);
            motor.setVelocityPIDFCoefficients(p, i, d, f);
            if(error > AcceptableError){
                p+= 0.03;
            } else if(errorD > 5){
                d+= 0.03;
            }
            previousVelocity = motor.getVelocity();
            previousTime = timer.time();
        }
    }
}
