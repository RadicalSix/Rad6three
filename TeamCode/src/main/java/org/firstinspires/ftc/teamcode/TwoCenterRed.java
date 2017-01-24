package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

/**

  This works

 */
//LinearOpMode
    //started 12/19 by Justin

@Autonomous(name = "TwoCenterRed", group = "Auto")

public class TwoCenterRed extends LinearOpMode {

    HardwarePushbotTDR robot = new HardwarePushbotTDR();
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
        public OneShootCenterRed(){

        }
    */
    @Override
    public void runOpMode() throws InterruptedException {


        robot.init(hardwareMap);

        double startPosL = robot.MotorL.getCurrentPosition();
        robot.LiftServo.setPosition(.05);
        robot.ShotFeeder.setPosition(.9);
        robot.CapGateServo.setPosition(0);//in
        robot.PressServoR.setPosition(1);//in
        robot.PressServoL.setPosition(0);//in
        robot.TouchServo.setPosition(0);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");
        telemetry.update();


        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        //shoot first ball then move servo so it can help hold up the ball..

        status = "Start, move servo";
        telemetry.addData("Status:", status);
        telemetry.update();

        startPosL = robot.MotorL.getCurrentPosition();
        status = "drive back until white line";
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() > startPosL - 3850 && !doneDrive1) {//stop if hit line or go certain distance
            if (robot.ColSensor.blue() > 8) {
                doneDrive1 = true;//hit white line
            }
            robot.MotorL.setPower(-.85 * vl);
            robot.MotorR.setPower(-.85 * vr);
            robot.TouchServo.setPosition(.55);
            robot.PressServoL.setPosition(0);//in
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition() - startPosL);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "drive back past line";
        startPosL = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(-.3 * vl);
        robot.MotorR.setPower(-.3 * vr);
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 650) && doneDrive1) {//only does this step if hit white line
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL units to go", robot.MotorL.getCurrentPosition() - startPosL + 650);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition());
            telemetry.update();
            idle();
        }

        status = "turn until white line";
        turnOneCount = 0;
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && robot.ColSensor.blue() < 20) {
            telemetry.addData("change in time", runtime.seconds() - lastClock);
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(-.34 * vl);
            robot.MotorR.setPower(.34 * vr);

            /*if(runtime.seconds() - lastClock > 1.7){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 200)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 200)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }*/
            telemetry.addData("lastPosL", lastPosL);
            telemetry.addData("current", robot.MotorL.getCurrentPosition());
            telemetry.addData("turnOneCount", turnOneCount);
        }


        status = "line follow";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        twolastPosL = 0;
        while (opModeIsActive() && !robot.TouSensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("TouSensor.isPressed()", robot.TouSensor.isPressed());
            telemetry.update();
            if (robot.ColSensor.blue() < 20) {//grey
                robot.MotorR.setPower(.2 * vl);
                robot.MotorL.setPower(-.4 * vr);
            } else if (robot.ColSensor.blue() > 20) {//white
                robot.MotorR.setPower(-.4 * vl);
                robot.MotorL.setPower(.2 * vr);
            }

            /*if(runtime.seconds() - lastClock > 2){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 400)//if neither motor has changed by more than 200, back up
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 400)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }*/
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData(" Change in last and current", Math.abs(lastPosL - robot.MotorL.getCurrentPosition()));
            telemetry.addData("followOneCount", followOneCount);
        }

        status = "sense color";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
            if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
                beaconOneRed = false;
            } else {
                beaconOneRed = true;
            }
        }

        status = "turn paddles";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        robot.TouchServo.setPosition(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .5) {
            telemetry.addData("Status:", status);
            telemetry.update();
            if (beaconOneRed) {
                robot.PressServoR.setPosition(0);//out
            } else {
                robot.PressServoL.setPosition(1);//in
            }
        }

        status = "backward and press buttons, beacon 1";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 300) && !beaconOneDone) {
            robot.MotorL.setPower(-.5 * vl);
            robot.MotorR.setPower(-.2 * vr);

            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL + 330);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconOneDone", beaconOneDone);
            telemetry.update();

            if(runtime.seconds() - lastClock > 1.7){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 90)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 90)){
                    beaconOneDone = true;
                }
            }
        }


        status = "forward off beacon 1";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 850) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(.6 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 1550);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn to beacon 2";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 1250) {
            robot.MotorL.setPower(.5 * vl);
            robot.MotorR.setPower(-.5 * vr);
            robot.PressServoR.setPosition(1);//in
            robot.PressServoL.setPosition(0);//in
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL + 1150);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "drive until white line";
        robot.TouchServo.setPosition(.55);
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 3500) && !longDriveDone) {//stop if hit line or go certain distance
            robot.MotorL.setPower(-.85 * vl);
            robot.MotorR.setPower(-.85 * vr);
            if (robot.ColSensor.blue() > 20) {
                longDriveDone = true;//hit white line
            }
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
        }
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);

        status = "backward past line";
        startPosL = robot.MotorL.getCurrentPosition();
        robot.MotorL.setPower(-.3 * vl);
        robot.MotorR.setPower(-.3 * vr);
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 350) && longDriveDone) {//stop if go certain distance or stuck on wall
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL units to go", robot.MotorL.getCurrentPosition() - startPosL + 550);
            telemetry.addData("MotorL current", robot.MotorL.getCurrentPosition());
            telemetry.addData("forwardTwoCount", forwardTwoCount);
            telemetry.addData("forwardTwoDone", forwardTwoDone);
            telemetry.update();
            lastPosL = robot.MotorL.getCurrentPosition();
        }

        status = "turn until white line, beacon 2";
        turnOneCount = 0;
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && robot.ColSensor.blue() < 8) {
            telemetry.addData("change in time", runtime.seconds() - lastClock);
            telemetry.addData("Status:", status);
            telemetry.update();
            robot.MotorL.setPower(-.35 * vl);
            robot.MotorR.setPower(.35 * vr);

            if(runtime.seconds() - lastClock > 2.7){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 200)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 200)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }
            telemetry.addData("lastPosL", lastPosL);
            telemetry.addData("current", robot.MotorL.getCurrentPosition());
            telemetry.addData("turnOneCount", turnOneCount);
        }


        status = "line follow, beacon 2";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && !robot.TouSensor.isPressed()) {
            telemetry.addData("Status:", status);
            telemetry.addData("TouSensor.isPressed()", robot.TouSensor.isPressed());
            telemetry.update();
            if (robot.ColSensor.blue() < 20) {//grey
                robot.MotorR.setPower(.2 * vl);
                robot.MotorL.setPower(-.4 * vr);
            } else if (robot.ColSensor.blue() > 20) {//white
                robot.MotorR.setPower(-.4 * vl);
                robot.MotorL.setPower(.2 * vr);
            }

            /*if(runtime.seconds() - lastClock > 2){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 200)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 200)){
                    while(robot.MotorL.getCurrentPosition() < lastPosL + 100){
                        robot.MotorL.setPower(.33 *vl);
                        robot.MotorR.setPower(.33 * vr);
                    }
                    robot.MotorL.setPower(0);
                    robot.MotorR.setPower(0);
                }
            }*/
            telemetry.addData("Time", runtime.seconds());
            telemetry.addData(" Change in last and current", Math.abs(lastPosL - robot.MotorL.getCurrentPosition()));
            telemetry.addData("followOneCount", followOneCount);
        }

        status = "sense color, beacon 2";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .1) {
            telemetry.addData("Status:", status);
            telemetry.addData("sensorColor:", robot.ColSensor.blue());
            telemetry.update();
            if (robot.FruitySensor.blue() > robot.FruitySensor.red()) {
                beaconOneRed = false;
            } else {
                beaconOneRed = true;
            }
        }

        status = "turn paddles, beacon 2";
        telemetry.update();
        robot.MotorR.setPower(0);
        robot.MotorL.setPower(0);
        robot.TouchServo.setPosition(0);
        startPosL = robot.MotorL.getCurrentPosition();
        runtime.reset();
        while (opModeIsActive() && runtime.seconds() < .7) {
            telemetry.addData("Status:", status);
            telemetry.update();
            if (beaconOneRed) {
                robot.PressServoR.setPosition(0);//out
            } else {
                robot.PressServoL.setPosition(1);//in
            }
        }

        status = "backward and press buttons, beacon 2";
        startPosL = robot.MotorL.getCurrentPosition();
        lastPosL = robot.MotorL.getCurrentPosition();
        lastPosR = robot.MotorR.getCurrentPosition();
        lastClock = runtime.seconds();
        while (opModeIsActive() && (robot.MotorL.getCurrentPosition() > startPosL - 260) && !beaconTwoDone) {
            robot.MotorL.setPower(-.5 * vl);
            robot.MotorR.setPower(-.5 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("currentPos - startPosL + 400", robot.MotorL.getCurrentPosition() - startPosL + 200);
            telemetry.addData("beaconOneCount", beaconOneCount);
            telemetry.addData("beaconTwoDone", beaconTwoDone);
            telemetry.update();

            if(runtime.seconds() - lastClock > 1.7){
                lastPosL = robot.MotorL.getCurrentPosition();
                lastPosR = robot.MotorR.getCurrentPosition();
                lastClock = runtime.seconds();
                if((Math.abs(lastPosL - robot.MotorL.getCurrentPosition()) < 90)
                        && (Math.abs(lastPosR - robot.MotorR.getCurrentPosition()) < 90)){
                    beaconTwoDone = true;
                }
            }
        }

        status = "forward off beacon 2";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 1000) {
            robot.MotorL.setPower(.4 * vl);
            robot.MotorR.setPower(.4 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 1000);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "turn to center";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 800) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(-.6 * vr);
            robot.PressServoR.setPosition(1);//in
            robot.PressServoL.setPosition(0);//out
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 800);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);

        status = "drive and park";
        telemetry.update();
        startPosL = robot.MotorL.getCurrentPosition();
        while (opModeIsActive() && robot.MotorL.getCurrentPosition() < startPosL + 4500) {
            robot.MotorL.setPower(.6 * vl);
            robot.MotorR.setPower(.6 * vr);
            telemetry.addData("Status:", status);
            telemetry.addData("MotorL to go", robot.MotorL.getCurrentPosition() - startPosL - 4500);
            telemetry.update();
        }
        robot.MotorL.setPower(0);
        robot.MotorR.setPower(0);
        }


    }



