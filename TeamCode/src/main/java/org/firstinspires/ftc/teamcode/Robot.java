package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Robot {

    private RRMecanumDrive drive;

    private Servo hand_servo;
    private Servo gripper_servo;
    private final double GRIPPERCLOSED = 0.7;
    private final double GRIPPEROPEN = 0.35;
    private DcMotorEx arm_motor;
    private DcMotorEx lift_motor;

    public Robot(HardwareMap hardwareMap){
        drive = new RRMecanumDrive(hardwareMap);
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
    }

    arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
    arm_motor.setTargetPosition(0);
    arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

}
