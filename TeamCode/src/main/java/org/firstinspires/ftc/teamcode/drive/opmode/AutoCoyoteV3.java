package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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

        waitForStart();

        telemetry.addLine("Running");
        telemetry.update();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(30)
                .build();

        drive.followTrajectory(traj1);
        Pose2d startPose = new Pose2d(-66,66, Math.PI/2);

        Trajectory traj2 = drive.trajectoryBuilder(traj1.end())
                .strafeRight(24)
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(traj2.end())
                .lineToSplineHeading(new Pose2d(24,0,-90))
                .build();

        Trajectory traj4 = drive.trajectoryBuilder(traj3.end().plus(new Pose2d(0, 0, Math.toRadians(90))), true)
                .splineToSplineHeading(new Pose2d(-24,24,Math.toRadians(90)),Math.toRadians(0))
                .build();

        drive.followTrajectory(traj2);
        drive.followTrajectory(traj3);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj4);

        ;
    }
}