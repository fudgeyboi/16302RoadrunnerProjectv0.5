package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import java.util.Arrays;
import java.util.List;

/**Primary autonomous OpMode*/
@Autonomous(group = "drive")
//@Disabled
public class BlueCoyote extends TeleOp {
    private DcMotorEx arm;
    private DcMotorEx slide;
    private List<DcMotorEx> motors;
    private double armpower;
    SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
    @Override
    public void runOpMode() throws InterruptedException {
        AutonomousInit();
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
            arm.setPower(armpower);
            drive.update();
        }
    }
    public void AutonomousInit() {
        //Retrieving hardware and naming it
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        launch = hardwareMap.get(Servo.class, "launch");
        liftkit = hardwareMap.get(DcMotor.class, "beesechurger");
        slide = hardwareMap.get(DcMotorEx.class, "slide");
        llaunch = hardwareMap.get(Servo.class, "llaunch");
        lift = hardwareMap.get(Servo.class, "lift");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        claw = hardwareMap.get(CRServo.class, "claw");
        eintake = hardwareMap.get(Servo.class, "eintake");
        motors = Arrays.asList(arm, slide);
        //Currently unknown as to the purpose of configuration
        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(0.95);
            motor.setMotorType(motorConfigurationType);
        }
        //Setting motor directions
        arm.setDirection(DcMotorEx.Direction.FORWARD);
        slide.setDirection(DcMotorEx.Direction.FORWARD);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Trajectory planning
        drive.setPoseEstimate(new Pose2d(-60, 12, Math.toRadians(0)));
        TrajectorySequence trajsq = drive.trajectorySequenceBuilder(new Pose2d(-60, -36, Math.toRadians(0)))
                .lineToSplineHeading(new Pose2d(-36, 48, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    armpower = 0.25;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    armpower = 0;
                    claw.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    claw.setPower(0);
                })
                .lineToSplineHeading(new Pose2d(-60, 48, Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(-60, 60, Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    eintake.setPosition(1);
                })
                .build();
        drive.followTrajectorySequenceAsync(trajsq);
        //Ready message
        telemetry.addLine("Ready");
        telemetry.update();
    }
}