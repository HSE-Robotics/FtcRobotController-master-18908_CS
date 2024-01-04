//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@TeleOp(group = "MightyHawks_RR")
public class MightyHawks_RR extends LinearOpMode {

    private Servo Wrist; //pa'l brazo
    private CRServo leftWheel;
    private CRServo rightWheel;
    private DcMotor ArmMotor;
    private Servo drone_launcher; // DroneServo
//    private Servo intake; //Intake Servo
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double sliderSpeed;
        double speed;
        int initialPosition;

       // slider = hardwareMap.get(DcMotor.class, "Slider");
        ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");

        ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        initialPosition = ArmMotor.getCurrentPosition();

        Wrist = hardwareMap.get(Servo.class, "Wrist");
        leftWheel = hardwareMap.get(CRServo.class,"leftTire");
        rightWheel = hardwareMap.get(CRServo.class,"rightTire");
//        intake = hardwareMap.get(CRServo.class,"IntakeServo");
        drone_launcher = hardwareMap.get(Servo.class, "PaperAirplane");

        // Put initialization blocks here.
        sliderSpeed = 0.5;
        waitForStart();
        if (opModeIsActive()) {

            speed = 0.7;
/*
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setDirection(DcMotor.Direction.REVERSE);
            initialPosition = slider.getCurrentPosition();

            slider.setTargetPosition((initialPosition)+224);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(sliderSpeed);
            */

            while (opModeIsActive() && !isStopRequested()) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speed,
                                gamepad1.left_stick_x * speed,
                                -gamepad1.right_stick_x * speed
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());


                if (gamepad1.share) {
                    drone_launcher.setPosition(0);
                }else if (gamepad1.options) {
                    drone_launcher.setPosition(0.5);
                }

                if ((gamepad1.left_trigger)>0) {
                   leftWheel.setPower(1);
                   rightWheel.setPower(-1);
                }else if ((gamepad1.right_trigger)>0) {
                    leftWheel.setPower(-1);
                    rightWheel.setPower(1);
                } else  {
                    leftWheel.setPower(0);
                    rightWheel.setPower(0);

                }


                if (gamepad1.a) {
                    ArmMotor.setTargetPosition(initialPosition + -683);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(sliderSpeed);
                } else if (gamepad1.x) {
                    ArmMotor.setTargetPosition(initialPosition + -1163);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(sliderSpeed);
                } else if (gamepad1.y) {
                    ArmMotor.setTargetPosition(initialPosition + -2526);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(sliderSpeed);
                } else if (gamepad1.b) {
                    ArmMotor.setTargetPosition(initialPosition);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(sliderSpeed);
                }

                if (gamepad1.start) {
                    initialPosition = ArmMotor.getCurrentPosition();
                }
                if (gamepad1.dpad_down) {
                    ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() - 100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(sliderSpeed);
                }
                if (gamepad1.dpad_up) {
                    ArmMotor.setTargetPosition(ArmMotor.getCurrentPosition() + 100);
                    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmMotor.setPower(sliderSpeed);
                }

                if (gamepad1.left_bumper) {
                    Wrist.setPosition(0.28);
                }else if (gamepad1.right_bumper){
                        Wrist.setPosition(0.8);
                }
               // telemetry.addData("gripper", gripper.getPosition());
               // telemetry.addData("gripper ", gripper.getPosition());
               // telemetry.addData("slider position", slider.getCurrentPosition());
                telemetry.addData("starting Pos.", initialPosition);
               telemetry.addData("Target Pos.", ArmMotor.getTargetPosition());
               telemetry.addData("Current Pos.", ArmMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }
}