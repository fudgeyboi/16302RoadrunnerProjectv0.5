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

    private DcMotorEx arm;
    public CRServo claw;
    public Servo launch;
    public DcMotor liftkit;
    public DcMotor slide;
    public Servo llaunch;
    public Servo lift;
    public VoltageSensor ControlHub_VoltageSensor;
    public Servo eintake;
    public CRServo intake;
    public CRServo counterroller;
    private int armExtend;
    private double leftstickxmod;
    private double rightstickxmod;
    private double leftstickymod;
    private double launchextend;

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
        claw = hardwareMap.get(CRServo.class, "claw");
        counterroller = hardwareMap.get(CRServo.class, "counterroller");
        armExtend = 0;
        launchextend = 0;

        // Put initialization blocks here.
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftkit.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launch.setDirection(Servo.Direction.FORWARD);
        arm.setVelocityPIDFCoefficients(2,2,3,1);
        arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        waitForStart();
        if (opModeIsActive()) {
            telemetry.setDisplayFormat(Telemetry.DisplayFormat.CLASSIC);
            eintake.setPosition(0);
            // Put run blocks here.
            while (opModeIsActive()) {
                leftstickxmod = gamepad1.left_stick_x*0.4;
                leftstickymod = gamepad1.left_stick_y*0.5;
                rightstickxmod = gamepad1.right_stick_x*0.5;
                // Put loop blocks here.
                drive.setWeightedDrivePower(
                        new Pose2d(
                                leftstickymod,
                                rightstickxmod,
                                -leftstickxmod
                        )
                    );

                drive.update();
                liftkit.setPower(-gamepad2.right_stick_y);
                slide.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                intake.setPower(gamepad2.right_trigger-gamepad2.left_trigger);
                counterroller.setPower(gamepad2.left_trigger-gamepad2.right_trigger);
                arm.setVelocity(120*gamepad2.left_stick_y,AngleUnit.DEGREES);
                if (gamepad2.left_bumper) {
                    claw.setPower(1);
                } else if (gamepad2.right_bumper) {
                    claw.setPower(-1);
                } else {
                    claw.setPower(0);
                }
                launchextend += gamepad1.left_trigger/150;
                if (launchextend > 1) {
                    launchextend = 1;
                } else if (launchextend < 0) {
                    launchextend = 0;
                }
                if (gamepad1.left_bumper) {
                    launchextend = 0;
                }
                llaunch.setPosition(launchextend);
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