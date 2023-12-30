package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

/*
 * Primary autonomous OpMode
 */
@Autonomous(group = "drive")
//@Disabled
public class AutoCoyoteV3 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        telemetry.addLine("Running");
        telemetry.update();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(30)
                .build();

        drive.followTrajectory(traj1);
        Pose2d startPose = new Pose2d(-66,66, Math.PI/2);

        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d())
                .lineToSplineHeading(new Pose2d(-66,42,0))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(new Pose2d(),true)
                .splineToSplineHeading(new Pose2d(6,6,Math.PI),0)
                .build();

        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj4);

        ;
    }
}