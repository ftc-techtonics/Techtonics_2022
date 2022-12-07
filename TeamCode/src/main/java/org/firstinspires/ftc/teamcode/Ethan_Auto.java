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

    // Distances made public so we can change in testing
    public static double DISTANCESTRAIGHT = 51;
    public static double DISTANCEBACK = 10;
    public static double DISTANCERIGHT = 17.5;
    public static double DISTANCELEFT = 17.5;

    // Motors, Servos, and Sensors
    private RRMecanumDrive drive;
    private TT_Camera camera;
    //private TT_TheAmazingLift lift;
    //private TT_ArmCode01 arm;
    private Servo hand;
    private Servo gripper;

    private int signalDetected = 0;
    private ElapsedTime timer = new ElapsedTime();
    private double gripperClosed = 0.7;
    private double gripperOpen = .35;

    DcMotorEx arm_motor;
    DcMotorEx lift_motor;

    @Override
    public void runOpMode() throws InterruptedException {
        // This enables viewing of the telemetry data through the browser Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // define variables and run the constructors
        drive = new RRMecanumDrive(hardwareMap);
        camera = new TT_Camera(hardwareMap);
        //lift = new TT_TheAmazingLift(hardwareMap);
        //arm = new TT_ArmCode01(hardwareMap);
        hand = hardwareMap.get(Servo.class, "hand");
        gripper = hardwareMap.get(Servo.class, "gripper");

        //initialize arm_motor stuff
        arm_motor = hardwareMap.get(DcMotorEx.class, "arm");
        //arm_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        arm_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        arm_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lift_motor = hardwareMap.get(DcMotorEx.class, "lift");
        //lift_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift_motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        lift_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //  After initilize is pressed, but before Play...  show what the camera detects.
        while (!isStarted()) {
            signalDetected = camera.scanSignal();
            telemetry.addData("Initialize tagID ", signalDetected);
            telemetry.update();
        }

        int lift_height = lift_motor.getCurrentPosition();
        telemetry.addLine("Lift_Height: " +  lift_height);
        telemetry.update();

        lift_motor.setTargetPosition(2500);
        lift_motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        lift_motor.setPower(0.3);

        lift_height =  lift_motor.getCurrentPosition();
        telemetry.addLine("Lift_Height: " +  lift_height);
        telemetry.update();

        sleep(6000);
        lift_motor.setTargetPosition(700);
        lift_motor.setPower(0.3);

        lift_height =  lift_motor.getCurrentPosition();
        telemetry.addLine("Lift_Height: " +  lift_height);
        telemetry.update();

    }
}
