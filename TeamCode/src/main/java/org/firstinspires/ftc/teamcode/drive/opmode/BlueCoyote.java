package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//Primary autonomous OpMode
@Autonomous(group = "drive")
//@Disabled
public class BlueCoyote extends TeleOp {
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
        claw = hardwareMap.get(CRServo.class, "claw");
        eintake = hardwareMap.get(Servo.class, "eintake");
        telemetry.addLine("Ready");
        telemetry.update();
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        drive.setPoseEstimate(new Pose2d(-60,12,Math.toRadians(0)));
        TrajectorySequence trajsq = drive.trajectorySequenceBuilder(new Pose2d(-60,12,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-36,48,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    arm.setPower(0.25);
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () ->{
                    arm.setPower(0);
                    claw.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () ->{
                    claw.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(-60,48,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-60,60,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    eintake.setPosition(1);
                })
                .build();

        drive.followTrajectorySequenceAsync(trajsq);

        /*AprilTagProcessor myAprilTagProcessor;

        AprilTagProcessor.Builder builder = new AprilTagProcessor.Builder();
        builder.setTagLibrary();
        builder.setDrawTagID(true);
        builder.setDrawTagOutline(true);
        builder.setDrawAxes(true);
        builder.setDrawCubeProjection(true);
        myAprilTagProcessor = builder
                .build();

        TfodProcessor myTfodProcessor;

        myTfodProcessor = new TfodProcessor.Builder()
                .setMaxNumRecognitions(10)
                .setUseObjectTracker(true)
                .setTrackerMaxOverlap((float) 0.2)
                .setTrackerMinSize(16)
                .build();

        VisionPortal myVisionPortal;

        myVisionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(myAprilTagProcessor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)
                .build();*/
        waitForStart();
        while (opModeIsActive()) {
            drive.update();
        }
    }
}