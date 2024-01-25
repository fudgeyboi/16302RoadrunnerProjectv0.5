package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

//Primary autonomous OpMode
@Autonomous(group = "drive")
//@Disabled
public class RedCoyote extends TeleOp {
    private DcMotorEx arm;
    private double armtarget;
    private double i;
    private double d;
    private double p;
    private double current_time;
    private double current_error;
    private double current_position;
    private double output;
    private double k_p;
    private double k_i;
    private double k_d;
    private double max_i;
    private double previous_time;
    private double previous_error;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
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
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        drive.setPoseEstimate(new Pose2d(60,12,Math.toRadians(180)));
        TrajectorySequence trajsq = drive.trajectorySequenceBuilder(new Pose2d(60,12,Math.toRadians(180)))
                .lineToSplineHeading(new Pose2d(36,48,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    armtarget = 500;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () ->{
                    claw.setPower(1);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () ->{
                    claw.setPower(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> {
                    armtarget = 10;
                })
                .lineToSplineHeading(new Pose2d(60,48,Math.toRadians(-90)))
                .lineToSplineHeading(new Pose2d(63,65,Math.toRadians(-90)))
                .UNSTABLE_addTemporalMarkerOffset(-3, () -> {
                    eintake.setPosition(1);
                })
                .build();

        drive.followTrajectorySequenceAsync(trajsq);
        k_d = 0;
        k_i = 0;
        k_p = 2;
        max_i = 1;

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
        resetRuntime();
        previous_time = 0;
        previous_error = 0;
        while(opModeIsActive()) {
            drive.update();
            current_time = getRuntime();
            current_error = armtarget - current_position;
            current_position = arm.getCurrentPosition();

            p = k_p * current_error;

            i += k_i * (current_error * (current_time - previous_time));

            if (i > max_i) {
                i = max_i;
            } else if (i < -max_i) {
                i = -max_i;
            }
            d = k_d * (current_error - previous_error) / (current_time - previous_time);

            output = p + i + d;

            previous_error = current_error;
            previous_time = current_time;
            telemetry.addData("Arm Power (requested)", output);
            arm.setPower(output);
            telemetry.update();
        };
    }
}
