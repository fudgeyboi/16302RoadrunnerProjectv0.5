package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    private DcMotorEx arm;
    private Servo launch;
    private DcMotor beesechurger;
    private DcMotor slide;
    private Servo llaunch;
    private Servo lift;
    private VoltageSensor ControlHub_VoltageSensor;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        arm = hardwareMap.get(DcMotorEx.class, "arm");
        launch = hardwareMap.get(Servo.class, "launch");
        beesechurger = hardwareMap.get(DcMotor.class, "beesechurger");
        slide = hardwareMap.get(DcMotor.class, "slide");
        llaunch = hardwareMap.get(Servo.class, "llaunch");
        lift = hardwareMap.get(Servo.class, "lift");
        ControlHub_VoltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        // Put initialization blocks here.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launch.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(DcMotorEx.Direction.REVERSE);
        arm.setPositionPIDFCoefficients(2);
        waitForStart();
        if (opModeIsActive()) {
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                if (gamepad1.right_stick_button && gamepad1.left_stick_button) {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                } else {
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.right_stick_x,
                                    -gamepad1.left_stick_x
                            )
                    );

                    drive.update();
                }
                beesechurger.setPower(gamepad1.right_stick_y);
                slide.setPower(gamepad2.right_stick_y);
                if (gamepad2.dpad_up) {
                    arm.setTargetPosition(800);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(Math.toRadians(720), AngleUnit.RADIANS);
                } else if (gamepad2.dpad_down) {
                    arm.setTargetPosition(40);
                    arm.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                    arm.setVelocity(Math.toRadians(720), AngleUnit.RADIANS);
                }
                if (gamepad1.right_trigger >= 0.9) {
                    launch.setPosition(0);
                }
                if (gamepad2.a) {
                    llaunch.setPosition(0.415);
                }
                if (gamepad2.b) {
                    llaunch.setPosition(0);
                }
                if (gamepad1.a) {
                    lift.setPosition(1);
                }
                if (gamepad1.b) {
                    lift.setPosition(0);
                }
                telemetry.addData("Arm Position (TICKS", arm.getCurrentPosition());
                telemetry.addData("Arm Current Draw", arm.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Voltage", ControlHub_VoltageSensor.getVoltage());
                if (gamepad1.back || gamepad2.back) {
                    requestOpModeStop();
                }
                telemetry.update();
            }
        }
    }
}