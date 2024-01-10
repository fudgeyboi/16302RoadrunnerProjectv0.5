package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
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
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        launch = hardwareMap.get(Servo.class, "launch");
        liftkit = hardwareMap.get(DcMotor.class, "beesechurger");
        slide = hardwareMap.get(DcMotor.class, "slide");
        llaunch = hardwareMap.get(Servo.class, "llaunch");
        lift = hardwareMap.get(Servo.class, "lift");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        telemetry.addLine("Ready");
        telemetry.update();
        drive.setPoseEstimate(new Pose2d(-60,12,Math.toRadians(0)));
        TrajectorySequence trajsq = drive.trajectorySequenceBuilder(new Pose2d(-60,12,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-36,48,Math.toRadians(-90)))
                .addDisplacementMarker(() -> {
                    arm.setTargetPosition(500);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(Math.toRadians(720), AngleUnit.RADIANS);
                })
                .addTemporalMarker(4,() -> {
                    claw.setPosition(0);
                    eintake.setPosition(1);
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
