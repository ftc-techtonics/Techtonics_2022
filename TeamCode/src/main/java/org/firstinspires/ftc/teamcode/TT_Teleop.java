package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(group = "Techtonics")
public class TT_Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RRMecanumDrive drive = new RRMecanumDrive(hardwareMap);
        double maxPower = .6;
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        TT_TheAmazingLift TAL = new TT_TheAmazingLift(hardwareMap);
        TT_ArmCode01 arm = new TT_ArmCode01(hardwareMap);
        int liftCurrentPosition = 0;

        waitForStart();

        while (!isStopRequested()) {
            /*
            drive.setWeightedDrivePower(
                    new Pose2d(
                            -Math.min(maxPower, gamepad1.left_stick_y),
                            -Math.min(maxPower, gamepad1.left_stick_x),
                            -Math.min(maxPower, gamepad1.right_stick_x)
                    )
            );
            drive.update();

             */
            liftCurrentPosition = TAL.Move(gamepad1.right_stick_y);
            int armCurrentPosition = arm.Move(gamepad1.left_stick_y);
            Pose2d poseEstimate = drive.getPoseEstimate();
            telemetry.addData("Lift   ", liftCurrentPosition);
            telemetry.addData("power ", gamepad1.right_stick_y);
            telemetry.addData("Lift   ", armCurrentPosition);
            telemetry.addData("power ", gamepad1.left_stick_y);
            telemetry.addLine();
            telemetry.update();

            /*
            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();
             */
        }
    }
}
