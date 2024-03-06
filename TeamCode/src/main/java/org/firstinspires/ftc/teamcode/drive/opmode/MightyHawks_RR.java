//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.drive.opmode;


import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;




@TeleOp(group = "MightyHawks_RR")
public class MightyHawks_RR extends LinearOpMode {

    private Servo Wrist; //pa'l brazo
   // private CRServo leftWheel;
    //private CRServo rightWheel;
    private DcMotor ArmRotator;
    //private Servo drone_launcher; // DroneServo
    private DcMotor SliderMotor;

    private Servo RServo;
    //private Servo claw;
    private Servo LServo;
    //private Servo ForearmServo;
//    private Servo intake; //Intake Servo
    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */

    public enum ManoState {
        MANO_INITIAL_TRANSPORT,
        MANO_AIM,
        MANO_PICKUP,
        MANO_SCORING,
        MANO_HANGING

    }
    ManoState manoState = ManoState.MANO_INITIAL_TRANSPORT;
    @Override
    public void runOpMode() {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        double sliderSpeed;
        double armSpeed;
        double speed;
        int initialPosition, initialPosition1, initialPositionForeArmMotor;


        SliderMotor = hardwareMap.get(DcMotor.class, "SliderMotor");
        ArmRotator = hardwareMap.get(DcMotor.class, "ArmRotator");
        RServo = hardwareMap.get(Servo.class, "RServo");
        LServo = hardwareMap.get(Servo.class, "LServo");
        Wrist = hardwareMap.get(Servo.class,"Wrist");

        // Put initialization blocks here.
        sliderSpeed = 0.5;
        armSpeed = 0.1;
        Wrist.setPosition(0.45);
        LServo.setDirection(Servo.Direction.REVERSE);
        RServo.setPosition(0.0);
        LServo.setPosition(0.0);
        waitForStart();

        if (opModeIsActive()) {
            initialPositionForeArmMotor = ArmRotator.getCurrentPosition();

            speed = 0.7;
            ArmRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ArmRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            //ArmRotator.setDirection(DcMotor.Direction.REVERSE);
            SliderMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            while (opModeIsActive() && !isStopRequested()) {

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y * speed,
                                -gamepad1.left_stick_x * speed,
                                -gamepad1.right_stick_x * speed
                        )
                );

                drive.update();

                Pose2d poseEstimate = drive.getPoseEstimate();
                telemetry.addData("x", poseEstimate.getX());
                telemetry.addData("y", poseEstimate.getY());
                telemetry.addData("heading", poseEstimate.getHeading());


                if (gamepad1.share) {
                    //drone_launcher.setPosition(1.0);
                }else if (gamepad1.options) {
                    //drone_launcher.setPosition(0.0);
                }

                if ((gamepad1.dpad_up)) {
                   Wrist.setPosition(0.75);
                }else if ((gamepad1.dpad_down)) {
                    Wrist.setPosition(0.45);
                }

                if (gamepad1.cross) {
                    AimForPixels(sliderSpeed, initialPositionForeArmMotor);
                    /*
                    ArmRotator.setTargetPosition(initialPositionForeArmMotor);
                    ArmRotator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    ArmRotator.setPower(0.15);

                    Wrist.setPosition(0.25);
                    RServo.setPosition(1.0);
                    LServo.setPosition(0.0);

                    while(ArmRotator.isBusy()){
                        if(gamepad1.ps){
                            ArmRotator.setPower(0.0);
                            break;
                        }
                    }

                     */

                }

                if (gamepad1.dpad_right) {
                    ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() + 10);
                    ArmRotator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    ArmRotator.setPower(0.05);
                    while(ArmRotator.isBusy()){
                        if(gamepad1.ps){
                            ArmRotator.setPower(0.0);
                            break;
                        }
                    }


                }
                if (gamepad1.dpad_left) {

                    ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() - 10);
                    ArmRotator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    ArmRotator.setPower(0.05);
                    while(ArmRotator.isBusy()){
                        if(gamepad1.ps){
                            ArmRotator.setPower(0.0);
                            break;
                        }
                    }

                }

                telemetry.addData("starting Pos ForeArm.", initialPositionForeArmMotor);
                telemetry.addData("Current Pos ForeArm.", ArmRotator.getCurrentPosition());
                telemetry.addData("Target Pos ForeArm.", ArmRotator.getTargetPosition());

                telemetry.addData("LeftServo",LServo.getPosition());
                telemetry.addData("RightServo",RServo.getPosition());
                telemetry.addData("STATE",manoState);
                telemetry.update();
            }
        }
    }

    public void Hanging(double sliderSpeed, int initialPositionForeArmMotor, int initialPosition1, int initialPosition){
        telemetry.addData("starting Pos.", SliderMotor.getCurrentPosition());
        telemetry.addData("Initial Pos", initialPositionForeArmMotor);

        SliderMotor.setTargetPosition(initialPosition1 + 5200);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);

        telemetry.addData("LeftSlider Target",SliderMotor.getTargetPosition());
        telemetry.update();
        while(SliderMotor.isBusy()){

        }
        manoState= ManoState.MANO_HANGING;
    }
    public void AimForPixels(double armSpeed, int initialPositionForeArmMotor){
        telemetry.addData("Current Pos.", ArmRotator.getCurrentPosition());
        telemetry.addData("TargetPosition", initialPositionForeArmMotor);

        SliderMotor.setTargetPosition(initialPositionForeArmMotor);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(armSpeed);
        while(ArmRotator.isBusy()){
            if(gamepad1.ps){
                ArmRotator.setPower(0.0);
                break;
            }
        }
        telemetry.update();


    }


    public void ForeArmRight(){

        //Set ForeArmServo to position
        ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() - 20);
        ArmRotator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        ArmRotator.setPower(0.75);
        while(ArmRotator.isBusy()){

        }
    }
    public void ForeArmLeft(){

        //Set ForeArmServo to position
        ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() + 20);
        ArmRotator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        ArmRotator.setPower(0.75);
        while(ArmRotator.isBusy()){

        }
    }
    public void SlidersUp(double sliderSpeed){
        telemetry.addData("starting Pos.", SliderMotor.getCurrentPosition());
        //telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

        SliderMotor.setTargetPosition(SliderMotor.getCurrentPosition() + 100);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);
        telemetry.addData("LeftSlider Target",SliderMotor.getTargetPosition());
        //telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
        telemetry.update();


    }
    public void SlidersDown(double sliderSpeed){
        telemetry.addData("starting Pos.", SliderMotor.getCurrentPosition());
        //telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

        SliderMotor.setTargetPosition(SliderMotor.getCurrentPosition() - 100);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);
        //Rightslider.setTargetPosition(Rightslider.getCurrentPosition() - 100);
        //Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        //Rightslider.setPower(sliderSpeed);
        telemetry.addData("LeftSlider Target",SliderMotor.getTargetPosition());
        //telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
        telemetry.update();

        }
    }


