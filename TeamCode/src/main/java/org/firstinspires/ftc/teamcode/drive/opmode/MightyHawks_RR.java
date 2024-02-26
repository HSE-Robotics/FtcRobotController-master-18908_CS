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
   // private CRServo leftWheel;
    //private CRServo rightWheel;
    private DcMotor ArmMotor;
    private DcMotor ForeArmMotor;
    //private Servo drone_launcher; // DroneServo
    private DcMotor leftSlider;
    private DcMotor Rightslider;
    private Servo RightsliderServo;
    private Servo claw;
    private Servo LeftSliderServo;
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
        double speed;
        int initialPosition, initialPosition1, initialPositionForeArmMotor;

       // slider = hardwareMap.get(DcMotor.class, "Slider");
        //ArmMotor = hardwareMap.get(DcMotor.class, "ArmMotor");
        leftSlider = hardwareMap.get(DcMotor.class, "leftSlider");
        Rightslider = hardwareMap.get(DcMotor.class, "Rightslider");
        ForeArmMotor = hardwareMap.get(DcMotor.class, "ForeArmMotor");
        RightsliderServo = hardwareMap.get(Servo.class, "RightsliderServo");
        LeftSliderServo = hardwareMap.get(Servo.class, "LeftSliderServo");
        claw = hardwareMap.get(Servo.class,"claw");
       // ForearmServo = hardwareMap.get(Servo.class, "ForearmServo");
        //ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
       // ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftSlider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Rightslider.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        ForeArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftSlider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Rightslider.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ForeArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        initialPosition = leftSlider.getCurrentPosition();
        initialPosition1 = Rightslider.getCurrentPosition();
        initialPositionForeArmMotor = ForeArmMotor.getCurrentPosition();
       /* ForeArmMotor.setTargetPosition((initialPositionForeArmMotor + 1));
        ForeArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        ForeArmMotor.setPower(0.5);*/
      //  Wrist = hardwareMap.get(Servo.class, "Wrist");
       // leftWheel = hardwareMap.get(CRServo.class,"leftTire");
       // rightWheel = hardwareMap.get(CRServo.class,"rightTire");
//        intake = hardwareMap.get(CRServo.class,"IntakeServo");
        //drone_launcher = hardwareMap.get(Servo.class, "PaperAirplane");

        // Put initialization blocks here.
        sliderSpeed = 0.5;
        waitForStart();
        claw.setPosition(1.0);
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
            leftSlider.setDirection(DcMotorSimple.Direction.REVERSE);
            LeftSliderServo.setDirection(Servo.Direction.REVERSE);


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
/*

                switch(manoState){
                    case MANO_INITIAL_TRANSPORT:
                        if(gamepad1.dpad_down){
                            //Set Sliders to aiming position
                            Aiming(sliderSpeed,initialPositionForeArmMotor, initialPosition1, initialPosition);

                            //manoState = ManoState.MANO_AIM;
                        }else if(gamepad1.dpad_up){

                            manoState = ManoState.MANO_SCORING;
                        }
                        break;
                    case MANO_AIM:
                        if(gamepad1.dpad_down){
                            Picking_Up(sliderSpeed, initialPositionForeArmMotor, initialPosition1, initialPosition);
                            //manoState = ManoState.MANO_PICKUP;
                        }else if(gamepad1.cross){

                            manoState = ManoState.MANO_INITIAL_TRANSPORT;
                        }
                        break;
                    case MANO_PICKUP:
                        if(gamepad1.dpad_up){

                            manoState = ManoState.MANO_INITIAL_TRANSPORT;
                        }else if(gamepad1.cross){

                            manoState = ManoState.MANO_AIM;
                        }
                        break;
                    case MANO_SCORING:

                        if(gamepad1.cross){
                            manoState = ManoState.MANO_INITIAL_TRANSPORT;
                        }
                        break;

                }
*/

                if ((gamepad1.left_trigger)>0) {
                   claw.setPosition(1);
                 //  rightWheel.setPower(-1);
                }else if ((gamepad1.right_trigger)>0) {
                    claw.setPosition(0);
                //    rightWheel.setPower(1);
                }
/*

                if (gamepad1.dpad_left) {
                    LeftSliderServo.setPosition(1);
                    RightsliderServo.setPosition(1);
                }
                if (gamepad1.dpad_right) {
                    LeftSliderServo.setPosition(0);
                    RightsliderServo.setPosition(0);
                }


                if (gamepad1.a) {
                    leftSlider.setTargetPosition(initialPosition1 + 683);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(initialPosition + 683);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    Rightslider.setPower(sliderSpeed);
                } else if (gamepad1.x) {
                    leftSlider.setTargetPosition(initialPosition1 + 1163);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(initialPosition + 1163);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    Rightslider.setPower(sliderSpeed);
                } else if (gamepad1.y) {
                    leftSlider.setTargetPosition(initialPosition1 + 2526);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(initialPosition + 2526);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    Rightslider.setPower(sliderSpeed);
                    //LeftSliderServo.setPosition(1);
                    //RightsliderServo.setPosition(1);
                    //ForearmServo.setPosition(1);

                } else if (gamepad1.b) {
                    leftSlider.setTargetPosition(initialPosition1);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(initialPosition);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    Rightslider.setPower(sliderSpeed);
                }
*/
                if (gamepad1.triangle) {
                    Hanging(sliderSpeed, initialPositionForeArmMotor, initialPosition1, initialPosition);
                    /*leftSlider.setTargetPosition(initialPosition1 + 4500);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(initialPosition + 4500);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    Rightslider.setPower(sliderSpeed);*/
                }


                if (gamepad1.square) {
                    Aiming(sliderSpeed, initialPositionForeArmMotor, initialPosition1, initialPosition);

                }
                if (gamepad1.dpad_right) {
                    //ForeArmRight();
                    //Set ForeArmServo to position
                    ForeArmMotor.setTargetPosition(ForeArmMotor.getCurrentPosition() + 35);
                    ForeArmMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    ForeArmMotor.setPower(1.0);


                }
                if (gamepad1.dpad_left) {
                    //ForeArmLeft();
                    //Set ForeArmServo to position
                    ForeArmMotor.setTargetPosition(ForeArmMotor.getCurrentPosition() - 35);
                    ForeArmMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    ForeArmMotor.setPower(1.0);

                }
                if (gamepad1.dpad_up) {
                    //SlidersUp(sliderSpeed);
                    telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
                    telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

                    leftSlider.setTargetPosition(leftSlider.getCurrentPosition() + 100);
                    Rightslider.setTargetPosition(Rightslider.getCurrentPosition() + 100);

                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));

                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setPower(sliderSpeed);

                    telemetry.addData("LeftSlider Target",leftSlider.getTargetPosition());
                    telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
                    telemetry.update();
                    while(Rightslider.isBusy() && leftSlider.isBusy()){

                    }
                }

                if (gamepad1.dpad_down) {
                    //SlidersUp(sliderSpeed);
                    telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
                    telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

                    leftSlider.setTargetPosition(leftSlider.getCurrentPosition() - 100);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(Rightslider.getCurrentPosition() - 100);
                    Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
                    Rightslider.setPower(sliderSpeed);
                    telemetry.addData("LeftSlider Target",leftSlider.getTargetPosition());
                    telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
                    telemetry.update();
                    while(Rightslider.isBusy() && leftSlider.isBusy()){

                    }
                }

                /*
                if (gamepad1.dpad_up) {
                    leftSlider.setTargetPosition(ArmMotor.getCurrentPosition() + 100);
                    leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftSlider.setPower(sliderSpeed);
                    Rightslider.setTargetPosition(ArmMotor.getCurrentPosition() + 100);
                    Rightslider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    Rightslider.setPower(sliderSpeed);
                }

                if (gamepad1.left_bumper) {
                    ForearmServo.setPosition(0.0);
                }else if (gamepad1.right_bumper){
                    ForearmServo.setPosition(0.8);
                }
                */
               // telemetry.addData("gripper", gripper.getPosition());
               // telemetry.addData("gripper ", gripper.getPosition());
               // telemetry.addData("slider position", slider.getCurrentPosition());
                telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
                telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());
                telemetry.addData("starting Pos ForeArm.", ForeArmMotor.getCurrentPosition());
                telemetry.addData("LeftServo",LeftSliderServo.getPosition());
                telemetry.addData("RightServo",RightsliderServo.getPosition());
                telemetry.addData("STATE",manoState);
               telemetry.addData("Target Pos.", ForeArmMotor.getTargetPosition());
               telemetry.addData("Current Pos.", ForeArmMotor.getCurrentPosition());
                telemetry.update();
            }
        }
    }

    public void Hanging(double sliderSpeed, int initialPositionForeArmMotor, int initialPosition1, int initialPosition){
        telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
        telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

        leftSlider.setTargetPosition(initialPosition1 + 5200);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setPower(sliderSpeed);
        Rightslider.setTargetPosition(initialPosition + 5200);
        Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        Rightslider.setPower(sliderSpeed);
        telemetry.addData("LeftSlider Target",leftSlider.getTargetPosition());
        telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
        telemetry.update();
        while(Rightslider.isBusy() && leftSlider.isBusy()){

        }
        //Set Slider Servos to position
        //LeftSliderServo.setPosition(0.0);
        //RightsliderServo.setPosition(0.0);
     /*   sleep(1500);

        //Set ForeArmServo to position
        ForeArmMotor.setTargetPosition(initialPositionForeArmMotor + 64);
        ForeArmMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        ForeArmMotor.setPower(0.25);
        while(ForeArmMotor.isBusy()){}
        claw.setPosition(1.0);*/
        manoState= ManoState.MANO_HANGING;
    }
    public void Aiming(double sliderSpeed, int initialPositionForeArmMotor, int initialPosition1, int initialPosition){
        telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
        telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

        leftSlider.setTargetPosition(initialPosition1 + 2827);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setPower(sliderSpeed);
        Rightslider.setTargetPosition(initialPosition + 2827);
        Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        Rightslider.setPower(sliderSpeed);
        telemetry.addData("LeftSlider Target",leftSlider.getTargetPosition());
        telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
        telemetry.update();
        while(Rightslider.isBusy() && leftSlider.isBusy()){

        }
        //Set Slider Servos to position
        //LeftSliderServo.setPosition(0.0);
        //RightsliderServo.setPosition(0.0);
     /*   sleep(1500);

        //Set ForeArmServo to position
        ForeArmMotor.setTargetPosition(initialPositionForeArmMotor + 64);
        ForeArmMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        ForeArmMotor.setPower(0.25);
        while(ForeArmMotor.isBusy()){}
        claw.setPosition(1.0);*/
        manoState= ManoState.MANO_AIM;
    }
    public void ForeArmRight(){

        //Set ForeArmServo to position
        ForeArmMotor.setTargetPosition(ForeArmMotor.getCurrentPosition() - 20);
        ForeArmMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        ForeArmMotor.setPower(0.75);
        while(ForeArmMotor.isBusy()){

        }
    }
    public void ForeArmLeft(){

        //Set ForeArmServo to position
        ForeArmMotor.setTargetPosition(ForeArmMotor.getCurrentPosition() + 20);
        ForeArmMotor.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        ForeArmMotor.setPower(0.75);
        while(ForeArmMotor.isBusy()){

        }
    }
    public void SlidersUp(double sliderSpeed){
        telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
        telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

        leftSlider.setTargetPosition(leftSlider.getCurrentPosition() + 100);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setPower(sliderSpeed);
        Rightslider.setTargetPosition(Rightslider.getCurrentPosition() + 100);
        Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        Rightslider.setPower(sliderSpeed);
        telemetry.addData("LeftSlider Target",leftSlider.getTargetPosition());
        telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
        telemetry.update();
        while(Rightslider.isBusy() && leftSlider.isBusy()){

        }
    }
    public void SlidersDown(double sliderSpeed){
        telemetry.addData("starting Pos.", leftSlider.getCurrentPosition());
        telemetry.addData("starting Pos1.", Rightslider.getCurrentPosition());

        leftSlider.setTargetPosition(leftSlider.getCurrentPosition() - 100);
        leftSlider.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftSlider.setPower(sliderSpeed);
        Rightslider.setTargetPosition(Rightslider.getCurrentPosition() - 100);
        Rightslider.setMode((DcMotor.RunMode.RUN_TO_POSITION));
        Rightslider.setPower(sliderSpeed);
        telemetry.addData("LeftSlider Target",leftSlider.getTargetPosition());
        telemetry.addData("RightSlider Target",Rightslider.getTargetPosition());
        telemetry.update();
        while(Rightslider.isBusy() && leftSlider.isBusy()){

        }
    }
}