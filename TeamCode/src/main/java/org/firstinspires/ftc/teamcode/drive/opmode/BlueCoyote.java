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
    private DcMotorEx arm;
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        eintake = hardwareMap.get(Servo.class, "eintake");
        //Setting motor directions
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //Trajectory planning
        drive.setPoseEstimate(new Pose2d(-60,12,Math.toRadians(0)));
        TrajectorySequence trajsq = drive.trajectorySequenceBuilder(new Pose2d(-60,12,Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-36,48,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-60,48,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-60,60,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-5, () -> {
                    eintake.setPosition(1);
                })
                .addTemporalMarker(() -> {
                    requestOpModeStop();
                })
                .build();
        drive.followTrajectorySequenceAsync(trajsq);
        //Ready message
        telemetry.addLine("Ready");
        telemetry.update();
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
        //Run section
        while (opModeIsActive()) {
            //Run loop
            drive.update();
        }
    }
}