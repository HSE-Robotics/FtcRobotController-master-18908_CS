//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode;


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

    private DcMotor slider;
    //private DcMotor intake; // intake

    private Servo box; // SliderServo
    private Servo drone_launcher; // DroneServo
//    private Servo intake; //Intake Servo
    private Servo claw; // Claw
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

        slider = hardwareMap.get(DcMotor.class, "Slider");
        //intake = hardwareMap.get(DcMotor.class, "intake");

        box = hardwareMap.get(Servo.class, "SliderServo");
        claw = hardwareMap.get(Servo.class,"claw");
//        intake = hardwareMap.get(CRServo.class,"IntakeServo");
        drone_launcher = hardwareMap.get(Servo.class, "DroneServo");

        box.scaleRange(0,1.00);
        // Put initialization blocks here.
        sliderSpeed = 0.5;
        waitForStart();
        if (opModeIsActive()) {

            speed = 0.7;

            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            slider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            slider.setDirection(DcMotor.Direction.REVERSE);
            initialPosition = slider.getCurrentPosition();

            slider.setTargetPosition((initialPosition)+224);
            slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            slider.setPower(sliderSpeed);

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

                if ((gamepad1.right_trigger)>0) {
                    box.setPosition(1);
                }else if ((gamepad1.left_trigger)>0) {
                    box.setPosition(0.05);
                }

                if (gamepad1.share) {
                    drone_launcher.setPosition(0);
                }else if (gamepad1.options) {
                    drone_launcher.setPosition(0.5);
                }

                if (gamepad1.left_bumper) {
                   claw.setPosition(1.0);
                }else if (gamepad1.right_bumper) {
                    claw.setPosition(0.25);
                }



                if (gamepad1.a) {
                    slider.setTargetPosition(initialPosition + 500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.x) {
                    slider.setTargetPosition(initialPosition + 1000);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.y) {
                    slider.setTargetPosition(initialPosition + 1500);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                } else if (gamepad1.b) {
                    slider.setTargetPosition(initialPosition);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
                if (gamepad1.start) {
                    initialPosition = slider.getCurrentPosition();
                }
                if (gamepad1.dpad_down) {
                    slider.setTargetPosition(slider.getCurrentPosition() - 100);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
                if (gamepad1.dpad_up) {
                    slider.setTargetPosition(slider.getCurrentPosition() + 100);
                    slider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    slider.setPower(sliderSpeed);
                }
               // telemetry.addData("gripper", gripper.getPosition());
               // telemetry.addData("gripper ", gripper.getPosition());
                telemetry.addData("slider position", slider.getCurrentPosition());
                telemetry.addData("starting Pos.", initialPosition);
                telemetry.addData("Target Pos.", slider.getTargetPosition());
                telemetry.update();
            }
        }
    }
}