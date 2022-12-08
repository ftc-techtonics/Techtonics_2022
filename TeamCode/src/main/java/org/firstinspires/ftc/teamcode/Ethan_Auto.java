package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.RRMecanumDrive;
import org.firstinspires.ftc.teamcode.TT_ArmCode01;
import org.firstinspires.ftc.teamcode.TT_Camera;
import org.firstinspires.ftc.teamcode.TT_TheAmazingLift;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(group = "Techtonics", name = "Ethan_Auto")
public class Ethan_Auto extends LinearOpMode {


    private Servo hand_servo;
    private Servo gripper_servo;
    private final double GRIPPERCLOSED = 0.7;
    private final double GRIPPEROPEN = 0.35;

    private DcMotorEx arm_motor;
    private DcMotorEx lift_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        // This enables viewing of the telemetry data through the browser Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        hand_servo = hardwareMap.get(Servo.class, "hand");
        gripper_servo = hardwareMap.get(Servo.class, "gripper");

        //initialize arm_motor stuff
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setTargetPosition(0);
        arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //intiialize lift_motor stuff
        lift_motor = hardwareMap.get(DcMotorEx.class, "lift");
        lift_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor.setTargetPosition(0);
        lift_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //  After initilize is pressed, but before Play...  show what the camera detects.
        while (!isStarted()) {
            //signalDetected = camera.scanSignal();
            //telemetry.addData("Initialize tagID ", signalDetected);
            //telemetry.update();
        }

        gripper_servo.setPosition(GRIPPERCLOSED);
        sleep(6000);
        gripper_servo.setPosition(GRIPPEROPEN);
        sleep(2000)

        /*
        hand_servo.setPosition(0);
        hand_servo.setPosition(0.5);
        sleep(3000);
        hand_servo.setPosition(0.8);
        sleep(3000);
         */

        /*
        arm_motor.setPower(0.5);
        arm_motor.setTargetPosition(400);
        */

        /*
        lift_motor.setPower(0.6);
        lift_motor.setTargetPosition(2500);

        int lift_height;
        while (lift_motor.isBusy()){
            lift_height = lift_motor.getCurrentPosition();
            telemetry.addData("lift_height:",lift_height);
            telemetry.update();
        }

        sleep(3000);
        lift_motor.setTargetPosition(700);
        while (lift_motor.isBusy()){
            lift_height = lift_motor.getCurrentPosition();
            telemetry.addData("lift_height:",lift_height);
            telemetry.update();
        }

        sleep(6000);
         */

    }
}
