package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

// Author:  me
// Team: 8947 - Techtonics
//bryce
//should work
//if doesnt work, please refer to https://www.youtube.com/watch?v=vR_E91r1Ya4
public class TT_ArmCode01 {
    private DcMotorEx motor;

    public TT_ArmCode01(HardwareMap hardwareMap) {
        motor = hardwareMap.get(DcMotorEx.class, "arm");
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int Move(double power) {
        if (power > 0.5) {         
            power = 0.5;
        } else if (power < -0.5) {
            power = -0.5;
        }
        if (power > 0 && motor.getCurrentPosition() >= 100) {
            power = 0;

        } else if (power < 0 && motor.getCurrentPosition() <= -900) {
            power = 0;
        }

        motor.setPower(power);
        return motor.getCurrentPosition();
    }
}

