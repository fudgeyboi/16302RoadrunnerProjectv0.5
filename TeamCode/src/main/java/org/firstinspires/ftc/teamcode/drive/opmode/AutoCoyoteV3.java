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
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Ready");
        telemetry.update();

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d(-60,12,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-36,48,Math.toRadians(-90)))
                .build();

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .lineToSplineHeading(new Pose2d(-60,48,Math.toRadians(-90)))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder((traj2.end()))
                .lineToSplineHeading(new Pose2d(-60,65,Math.toRadians(-90)))
                .build();

        waitForStart();

        telemetry.addLine("Running");
        telemetry.update();

        if (isStopRequested()) return;
        drive.setPoseEstimate(new Pose2d(-60,12,Math.toRadians(0)));

        drive.followTrajectory(traj1);
        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        ;
    }
}