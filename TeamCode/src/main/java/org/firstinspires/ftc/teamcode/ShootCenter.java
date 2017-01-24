package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by Troy on 10/01/16.
  This works

 */
//LinearOpMode

//@Autonomous(name = "ShootCenter", group = "Auto")

public class ShootCenter extends LinearOpMode{

    HardwarePushbotTDR         robot   = new HardwarePushbotTDR();

    VuforiaOp camera = new VuforiaOp();
    private ElapsedTime runtime = new ElapsedTime();
    Boolean beaconOneRed;
    Boolean beaconTwoRed;
    Boolean doneDrive1 = false;
    Boolean beaconOneDone = false;
    Boolean beaconTwoDone = false;
    Boolean forwardTwoDone = false;
    Boolean turnTwoDone = false;
    Boolean longDriveDone = false;
    Boolean followOneDone = false;


    double vr = 1;//change for direction and battery
    double vl = 1;//change for direction and battery
    double shotSpeed = .39;//change for battery
    double startPosL = 0;
    int step = 0;
    double shot = 0;
    double lastPosL = 0;
    double lastPosR = 0;
    double twolastPosL = 0;
    double lastClock = 0;
    int beaconOneCount = 0;
    int BeaconTwoCount = 0;
    int forwardTwoCount = 0;
    int turnTwoCount = 0;
    int followOneCount = 0;
    int turnOneCount = 0;
    String status = "Start";

/*
    public TestRun(){

    }
*/
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        robot.LiftServo.setPosition(.05);//down
        robot.ShotFeeder.setPosition(.9);//down
        robot.CapGateServo.setPosition(0);//in
        robot.PressServoR.setPosition(1);//in
        robot.PressServoL.setPosition(0);//in
        robot.TouchServo.setPosition(0);//in

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        telemetry.addData("Start", 0);
        telemetry.update();

        //turn on shooter
        status = "shooter on";
        runtime.reset();
        while (opModeIsActive() && shot < shotSpeed) {
            shot += 0.02;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
        }

        robot.ShooterDown.setPower(shotSpeed);
        robot.ShooterUp.setPower(-shotSpeed);


        status = "shoot first ball";
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shot = shotSpeed;
            robot.ShotFeeder.setPosition(0);//shoot
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();

            robot.CapGateServo.setPosition(1);//out
        }

        status = "feed second ball";
        robot.TouchServo.setPosition(.15);//out so paddles don't hit
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 2) {
            robot.PressServoL.setPosition(.6);//out part of the way
            robot.PressServoR.setPosition(.4);//out part of the way
            robot.TouchServo.setPosition(0);
            robot.ShotFeeder.setPosition(.9);//down
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();

            robot.Conveyor.setPower(.7);
        }
        robot.Conveyor.setPower(0);
        robot.CapGateServo.setPosition(0);//in
        robot.TouchServo.setPosition(0);//in

        status = "shoot second ball";
        robot.Conveyor.setPower(0);
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < 1.5) {
            shot = shotSpeed;
            robot.ShotFeeder.setPosition(0);//shoot
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
            telemetry.addData("shot", shot);
            telemetry.addData("Path", "Leg 1: %2.5f S Elapsed", runtime.seconds());
            telemetry.addData("Status:", status);
            telemetry.update();
        }

        status = "shooter off";
        runtime.reset();
        while (opModeIsActive() && shot > .18) {
            shot -= 0.02;
            robot.ShotFeeder.setPosition(.9);
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
        }

        status = "shooter off";
        runtime.reset();
        while (opModeIsActive() && shot > 0) {
            shot -= 0.0025;
            robot.ShooterDown.setPower(shot);
            robot.ShooterUp.setPower(-shot);
        }

        robot.ShooterDown.setPower(0);
        robot.ShooterUp.setPower(0);

        status = "forward off beacon 2";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 3000) {
            robot.MotorL.setPower(.4 * vl);
            robot.MotorR.setPower(.4 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 1000);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);


    }
}