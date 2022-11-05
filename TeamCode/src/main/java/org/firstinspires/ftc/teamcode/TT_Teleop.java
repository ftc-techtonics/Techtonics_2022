package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@TeleOp(group = "Techtonics")
public class TT_Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);
        double maxPower = .6;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();
        TT_TheAmazingLift TAL=new TT_TheAmazingLift(hardwareMap);
        int liftCurrentPosition = 0;


        while (!isStopRequested()) {
            liftCurrentPosition = TAL.Move(gamepad1.right_stick_y);
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -Math.min(maxPower, gamepad1.left_stick_y),
                            -Math.min(maxPower, gamepad1.left_stick_x),
                            -Math.min(maxPower, gamepad1.right_stick_x)
                    )
            );

            drive.update();

            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Lift ", liftCurrentPosition);
            telemetry.addLine();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
        }
    }
}
