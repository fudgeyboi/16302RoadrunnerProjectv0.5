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

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj1 = drive.trajectoryBuilder(new Pose2d())
                .strafeLeft(30)
                .build();

        drive.followTrajectory(traj1);
        Pose2d startPose = new Pose2d(6, 6, Math.toRadians(90));


        Trajectory traj2 = drive.trajectoryBuilder(new Pose2d())
                .strafeRight(24)
                .splineToSplineHeading(new Pose2d(24,36, Math.toRadians(-90)), Math.toRadians(-90))
                .build();

        Trajectory traj3 = drive.trajectoryBuilder(new Pose2d(),true)
                .splineToConstantHeading(new Vector2d(6,6), Math.toRadians(90))
                .build();
        drive.followTrajectory(traj2);
        drive.turn(Math.toRadians(90));
        drive.followTrajectory(traj3);

        ;
    }
}
