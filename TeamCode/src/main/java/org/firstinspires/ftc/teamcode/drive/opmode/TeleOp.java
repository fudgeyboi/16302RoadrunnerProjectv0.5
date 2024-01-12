package org.firstinspires.ftc.teamcode.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    public DcMotorEx arm;
    public Servo claw;
    public Servo launch;
    public DcMotor liftkit;
    public DcMotor slide;
    public Servo llaunch;
    public Servo lift;
    public VoltageSensor ControlHub_VoltageSensor;
    public Servo eintake;
    public CRServo intake;
    private int armExtend;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
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
        eintake = hardwareMap.get(Servo.class, "eintake");
        intake = hardwareMap.get(CRServo.class, "intake");
        claw = hardwareMap.get(Servo.class, "claw");
        armExtend = 0;

        // Put initialization blocks here.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        launch.setDirection(Servo.Direction.FORWARD);
        arm.setVelocityPIDFCoefficients(2,2,3,1);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
            eintake.setPosition(0);
            // Put run blocks here.
            while (opModeIsActive()) {
                // Put loop blocks here.
                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.right_stick_x,
                                -gamepad1.left_stick_x
                        )
                    );

                drive.update();
                liftkit.setPower(-gamepad2.right_stick_y);
                slide.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                arm.setVelocity(120*gamepad2.left_stick_y,AngleUnit.DEGREES);
                /*armExtend += 50*gamepad2.left_stick_y;
                if (armExtend > 750) {
                    armExtend = 750;
                } else if (armExtend <= 10) {
                    armExtend = 10;
                }
                arm.setTargetPosition(armExtend);
                arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                arm.setVelocity(Math.toRadians(1080), AngleUnit.RADIANS);*/

                if (gamepad1.right_trigger >= 0.9) {
                    launch.setPosition(0);
                }
                if (gamepad2.right_bumper) {
                    claw.setPosition(0.95);
                }
                if (gamepad2.left_bumper) {
                    claw.setPosition(1);
                }
                if (gamepad1.a) {
                    llaunch.setPosition(0.415);
                }
                if (gamepad1.b) {
                    llaunch.setPosition(0);
                }
                if (gamepad2.a) {
                    lift.setPosition(1);
                }
                if (gamepad2.b) {
                    lift.setPosition(0);
                }
                telemetry.addData("Arm Position (TICKS)", arm.getCurrentPosition());
                telemetry.addData("Arm Current Draw", arm.getCurrent(CurrentUnit.AMPS));
                telemetry.addData("Voltage", ControlHub_VoltageSensor.getVoltage());
                telemetry.addData("ArmVar", armExtend);
                if (gamepad1.back || gamepad2.back) {
                    requestOpModeStop();
                }
                telemetry.update();
            }
        }
    }
}