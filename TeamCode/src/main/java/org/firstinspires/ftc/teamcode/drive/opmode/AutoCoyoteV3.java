package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/*
 * Primary autonomous OpMode
 */
@Autonomous(group = "drive")
//@Disabled
public class AutoCoyoteV3 extends TeleOp {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addLine("Ready");
        telemetry.update();

        TrajectorySequence trajsq = drive.trajectorySequenceBuilder(new Pose2d(-60,12,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-36,48,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    arm.setTargetPosition(500);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(Math.toRadians(720), AngleUnit.RADIANS);
                })
                .addTemporalMarker(() -> {
                    claw.setPosition(0);
                })
                .lineToSplineHeading(new Pose2d(-60,48,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-60,65,Math.toRadians(-90)))
                .build();

        drive.followTrajectorySequenceAsync(trajsq);
        waitForStart();
        while(opModeIsActive()){
            drive.update();
        };
    }
}
