//package org.firstinspires.ftc.teamcode;
package org.firstinspires.ftc.teamcode.drive.opmode;


import android.transition.Slide;

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
    private Servo drone_launcher; // DroneServo
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
        boolean leftServoOpen, rightServoOpen, wristLastInput, wristCurrentInput, wristPositionDown;
        int sliderInitialPosition, initialPosition1, initialPositionForeArmMotor;


        SliderMotor = hardwareMap.get(DcMotor.class, "SliderMotor");
        ArmRotator = hardwareMap.get(DcMotor.class, "ArmRotator");
        RServo = hardwareMap.get(Servo.class, "RServo");
        LServo = hardwareMap.get(Servo.class, "LServo");
        Wrist = hardwareMap.get(Servo.class,"Wrist");
        drone_launcher = hardwareMap.get(Servo.class,"drone");

        // Put initialization blocks here.
        sliderSpeed = 0.75;
        armSpeed = 1.0;
        Wrist.setPosition(0.45);
        LServo.setDirection(Servo.Direction.REVERSE);
        RServo.setPosition(0.75);
        LServo.setPosition(0.75);
        leftServoOpen = false;
        rightServoOpen = false;
        wristLastInput = false;
        wristCurrentInput = false;
        wristPositionDown = false;
        waitForStart();


        if (opModeIsActive()) {
            initialPositionForeArmMotor = ArmRotator.getCurrentPosition();
            sliderInitialPosition = SliderMotor.getCurrentPosition();

            speed = 0.85;
           // ArmRotator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            SliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            SliderMotor.setDirection(DcMotor.Direction.REVERSE);

            ArmRotator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ArmRotator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            ArmRotator.setDirection(DcMotor.Direction.FORWARD);


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


                if (gamepad1.options) {
                    drone_launcher.setPosition(0.0);
                }
                if (gamepad1.share) {
                    initialPositionForeArmMotor = ArmRotator.getCurrentPosition();
                    sliderInitialPosition = SliderMotor.getCurrentPosition();
                }

                if (gamepad1.touchpad){
                    Wrist.setPosition(0.0);
                }

                if (gamepad1.left_bumper) {
                    LServo.setPosition(0.25);

                }
                if (gamepad1.right_bumper) {
                    RServo.setPosition(.25);
                }
                if (gamepad1.right_trigger > 0.65){
                    RServo.setPosition(0.25);
                    LServo.setPosition(0.25);
                }
                if (gamepad1.left_trigger > 0.65){
                    RServo.setPosition(0.75);
                    LServo.setPosition(0.85);
                }
                wristLastInput = wristCurrentInput;
                wristCurrentInput = gamepad1.triangle;
                if ((wristLastInput && !wristCurrentInput)) {
                    wristPositionDown = !wristPositionDown;
                    if(wristPositionDown){
                        Wrist.setPosition(0.75);
                    }else{
                        Wrist.setPosition(0.45);
                    }

                }
                if (gamepad1.dpad_down) {
                    SliderMotor.setTargetPosition(SliderMotor.getCurrentPosition() - 250);
                    SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderMotor.setPower(0.35);

                    while(SliderMotor.isBusy()){
                        if(gamepad1.ps){
                            SliderMotor.setPower(0.0);
                            SliderMotor.setTargetPosition(SliderMotor.getCurrentPosition());
                            SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SliderMotor.setPower(sliderSpeed);
                            break;
                        }
                    }


                }
                if (gamepad1.dpad_up) {
                    SliderMotor.setTargetPosition(SliderMotor.getCurrentPosition() + 250);
                    SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    SliderMotor.setPower(0.35);
                    while(SliderMotor.isBusy()){
                        if(gamepad1.ps){
                            SliderMotor.setPower(0.0);
                            SliderMotor.setTargetPosition(SliderMotor.getCurrentPosition());
                            SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                            SliderMotor.setPower(0.25);
                            break;
                        }
                    }
                }

                if (gamepad1.square) {
                    AimForScoring(armSpeed, initialPositionForeArmMotor,3700, 0.35, sliderInitialPosition, 1000);

                }

                if (gamepad1.cross) {
                    AimForPixels(armSpeed, initialPositionForeArmMotor, sliderSpeed, sliderInitialPosition);
                }

                if (gamepad1.circle) {
                    Hanging(armSpeed, sliderSpeed,  initialPositionForeArmMotor, sliderInitialPosition);
                }


                if (gamepad1.dpad_right && ((ArmRotator.getCurrentPosition() + 250) < (initialPositionForeArmMotor+ 7000))) {
                    ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() + 250);
                    ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    ArmRotator.setPower(0.75);
                    while(ArmRotator.isBusy()){
                        if(gamepad1.ps){
                            ArmRotator.setPower(0.0);
                            break;
                        }
                    }


                }
                if (gamepad1.dpad_left) {

                    ArmRotator.setTargetPosition(ArmRotator.getCurrentPosition() - 250);
                    ArmRotator.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    ArmRotator.setPower(0.75);
                    while(ArmRotator.isBusy()){
                        if(gamepad1.ps){
                            ArmRotator.setPower(0.0);
                            break;
                        }
                    }

                }


                telemetry.addData("starting Pos ForeArm.", initialPositionForeArmMotor);
                telemetry.addData("starting Pos Slider.", sliderInitialPosition);
                telemetry.addData("Current Pos ForeArm.", ArmRotator.getCurrentPosition());
                telemetry.addData("Target Pos ForeArm.", ArmRotator.getTargetPosition());
                telemetry.addData("Slider Pos", SliderMotor.getCurrentPosition());

                telemetry.addData("LeftServo",LServo.getPosition());
                telemetry.addData("RightServo",RServo.getPosition());
                telemetry.addData("STATE",manoState);
                telemetry.update();
            }
        }
    }

    public void Hanging(double armSpeed, double sliderSpeed, int initialPositionForeArmMotor, int sliderInitialPosition) {
        telemetry.addData("starting Pos.", SliderMotor.getCurrentPosition());
        telemetry.addData("Initial Pos", initialPositionForeArmMotor);

        SliderMotor.setTargetPosition(sliderInitialPosition + 4500);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);


        ArmRotator.setTargetPosition(initialPositionForeArmMotor + 6500);
        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmRotator.setPower(armSpeed);
        while (ArmRotator.isBusy()) {
            telemetry.addData("Current Pos.", ArmRotator.getCurrentPosition());
            telemetry.addData("TargetPosition", ArmRotator.getTargetPosition());
            telemetry.update();
            if (gamepad1.ps) {
                ArmRotator.setPower(0.0);

            }
            idle();

            manoState = ManoState.MANO_HANGING;
        }
    }
    public void AimForPixels(double armSpeed, int initialPositionForeArmMotor, double sliderSpeed, int sliderInitialPosition){
        //Cross
        SliderMotor.setTargetPosition(sliderInitialPosition);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);
            if(gamepad1.ps){
                SliderMotor.setPower(0.0);

            }
            idle();
        ArmRotator.setTargetPosition(initialPositionForeArmMotor);
        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmRotator.setPower(armSpeed);
            if(gamepad1.ps){
                ArmRotator.setPower(0.0);

            }



        Wrist.setPosition(0.45);
        RServo.setPosition(0.5);
        LServo.setPosition(0.5);
    }
    public void AimForScoring(double armSpeed, int initialPositionForeArmMotor, int scoringPosition, double sliderSpeed, int sliderInitialPosition, int sliderTargetPosition){
        telemetry.addData("Current Pos.", ArmRotator.getCurrentPosition());
        telemetry.addData("TargetPosition", ArmRotator.getTargetPosition());
        telemetry.update();
        ArmRotator.setTargetPosition(initialPositionForeArmMotor + scoringPosition);
        ArmRotator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ArmRotator.setPower(armSpeed);
            if(gamepad1.ps){
                ArmRotator.setPower(0.0);
            }
            idle();


        SliderMotor.setTargetPosition(sliderInitialPosition + sliderTargetPosition);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);
            if(gamepad1.ps){
                SliderMotor.setPower(0.0);
            }

        Wrist.setPosition(0.0);

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
    public void SlidersUp(double sliderSpeed, int sliderInitialPosition){
        telemetry.addData("starting Pos.", SliderMotor.getCurrentPosition());

        SliderMotor.setTargetPosition(sliderInitialPosition + 2411);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);
        telemetry.addData("Slider Target",SliderMotor.getTargetPosition());
        telemetry.update();
        while(SliderMotor.isBusy()){
            telemetry.addData("Slider Current Pos.", SliderMotor.getCurrentPosition());
            telemetry.addData("Slider Target",SliderMotor.getTargetPosition());
            telemetry.update();
        }


    } public void SlidersUpToScore(double sliderSpeed, int sliderInitialPosition, int sliderTargetPosition){
        telemetry.addData("starting Pos.", SliderMotor.getCurrentPosition());

        SliderMotor.setTargetPosition(sliderInitialPosition + sliderTargetPosition);
        SliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        SliderMotor.setPower(sliderSpeed);
        telemetry.addData("Slider Target",SliderMotor.getTargetPosition());
        telemetry.update();
        while(SliderMotor.isBusy()){
            telemetry.addData("Slider Current Pos.", SliderMotor.getCurrentPosition());
            telemetry.addData("Slider Target",SliderMotor.getTargetPosition());
            telemetry.update();
        }


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


