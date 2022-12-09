package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;

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

    public Robot(HardwareMap hardwareMap) {
        //assign the motors
        drive = new RRMecanumDrive(hardwareMap);
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        lift_motor = hardwareMap.get(DcMotorEx.class, "lift");
        hand_servo = hardwareMap.get(Servo.class, "hand");
        gripper_servo = hardwareMap.get(Servo.class, "gripper");

        //prep the motors
        //arm
        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //lift
        lift_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor.setTargetPosition(0);
        lift_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //not tested yet
    public void prepToGrabCone(String side){
        if (side.equals("left")){
            arm_motor.setTargetPosition(-100);
            hand_servo.setPosition(0.8);
        }else{
            arm_motor.setTargetPosition(-1000);
            hand_servo.setPosition(0.3);
        }
    }

}
