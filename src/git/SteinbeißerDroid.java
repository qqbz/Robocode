/**
 * Copyright (c) 2001-2014 Mathew A. Nelson and Robocode contributors All rights
 * reserved. This program and the accompanying materials are made available
 * under the terms of the Eclipse Public License v1.0 which accompanies this
 * distribution, and is available at
 * http://robocode.sourceforge.net/license/epl-v10.html
 */
package git;

import static robocode.util.Utils.normalRelativeAngleDegrees;
import java.util.Hashtable;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.logging.Level;
import java.util.logging.Logger;
import robocode.*;
import robocode.util.Utils;
import robocode.MessageEvent;
import robocode.TeamRobot;
import static robocode.util.Utils.normalRelativeAngleDegrees;

public class SteinbeißerDroid extends TeamRobot implements Droid {

    public Hashtable<String, RobotInfo> robotFileList = new Hashtable<String, RobotInfo>();
    SteinbeißerLeader leader = new SteinbeißerLeader();
    RobotInfo robotFile = new RobotInfo();
    String myNameIs = "git.SteinbeißerDroid";//
    String target = "sample.Crazy";// zum testen da noch kein target
//    String target = leader.getTarget();

    public void run() {

        while (true) {

            execute();

        }

    }

    /**
     * onMessageReceived: What to do when our leader sends a message
     */
    public void onMessageReceived(MessageEvent e) {
        robotFile = (RobotInfo) e.getMessage();
        String scanedRobotName = robotFile.NAME;
        if (!isTeammate(scanedRobotName)) {
            if (!robotFileList.containsKey(scanedRobotName)) {
                robotFileList.put(scanedRobotName, robotFile);

            }
        }
        robotFileList.replace(scanedRobotName, robotFile);
        if (target.equals(robotFile.NAME)) {
            goTo(robotFileList.get(target));
        }
    }

    private void goTo(Point2D destination) {
        //http://www.jasonsjava.com/?cat=2
//		stop(); // don't move again until we are ready
        Point2D location = new Point2D.Double(getX(), getY());
        System.out.println("JETZIGE POSITION " + location);
        double distance = location.distance(destination);
        // this angle is the amount I need to turn right to face the destination point
        double angle = Utils.normalRelativeAngleDegrees(Math.toDegrees(Math.atan2(destination.getX() - location.getX(), destination.getY() - location.getY())) - getHeading());
        // this statement takes the previous angle and basically divides it in half.
        // if my rear end is closer, then turn the rear towards the destination and drive backwards.
        if (Math.abs(angle) > 90) {
            distance *= -1;  // drive backwards
            if (angle > 0) {
                angle -= 180;
            } else {
                angle += 180;
            }
        }

        out.println("Going to: " + destination);
        turnRight(angle); // complete the turn before going any distance (blocking call from Robot class)
        setAhead(distance);

        if (distance <= 200) {
            //vor dem schießen muss noch Winkel bestimmt werden
            fire(1.5);
            execute();
        }

        // must be called because setAhead() will not move without a call to execute().
    }

    double absoluteBearing(double x1, double y1, double x2, double y2) {
        double xo = x2 - x1;
        double yo = y2 - y1;
        double hyp = Point2D.distance(x1, y1, x2, y2);
        double arcSin = Math.toDegrees(Math.asin(xo / hyp));
        double bearing = 0;

        if (xo > 0 && yo > 0) { // both pos: lower-Left
            bearing = arcSin;
        } else if (xo < 0 && yo > 0) { // x neg, y pos: lower-right
            bearing = 360 + arcSin; // arcsin is negative here, actuall 360 - ang
        } else if (xo > 0 && yo < 0) { // x pos, y neg: upper-left
            bearing = 180 - arcSin;
        } else if (xo < 0 && yo < 0) { // both neg: upper-right
            bearing = 180 - arcSin; // arcsin is negative here, actually 180 + ang
        }

        return bearing;
    }

    @Override
    public void onHitRobot(HitRobotEvent e) {
        if (isTeammate(e.getName())) {
            turnLeft(e.getBearing() + 90);
            ahead(100);
            System.out.println("Leader getroffen");
        }

    }

    @Override
    public void onHitWall(HitWallEvent event) {
        out.println("Ich habe die Wand getroffen " + event.getBearing() + "180");
        turnLeft(event.getBearing() + 45);
        ahead(120);
    }
//
//    private void move() {
//
//        double height = this.getBattleFieldHeight();
//        double wide = this.getBattleFieldWidth();
////        double yBuffer = DANGER_ZONE * height;
//        double yBuffer = 120;
//        double xBuffer = 120;
//        double turnAngle = 45;
//
//        double xPosition = this.getX();
//        double yPosition = this.getY();
//        double direction = this.getHeading();
////        System.out.println("Ich fahre");
////        out.println(" die Position X=" + xPosition + " Y=" + yPosition);
//
//        if ((yPosition < yBuffer)) {
//
////            System.out.println("Gefahr");
//            if ((this.getHeading() < 180) && (this.getHeading() > 90)) {
//                setAhead(5);
//                this.setTurnLeft(this.getHeading() - 90);
//                execute();
//            } else if ((this.getHeading() < 270) && (this.getHeading() > 180)) {
//                setAhead(5);
//                this.setTurnRight(270 - this.getHeading());
//                execute();
//
//            }
//
//        } else if (yPosition > height - yBuffer) {
////            System.out.println("Gefahr");
//            if ((this.getHeading() < 90) && (this.getHeading() > 0)) {
//                setAhead(5);
//                this.setTurnRight(90 - this.getHeading());
//                execute();
//            } else if ((this.getHeading() < 360) && (this.getHeading() > 270)) {
//                setAhead(5);
//                this.setTurnLeft(this.getHeading() - 270);
//                execute();
//            }
//        } else if (xPosition < xBuffer) {
////            System.out.println("Gefahr");
//            if ((this.getHeading() > 180) && (this.getHeading() < 270)) {
//                setAhead(5);
//                this.setTurnLeft(this.getHeading() - 180);
//                execute();
//            } else if ((this.getHeading() > 270) && (this.getHeading() < 360)) {
//                setAhead(5);
//                this.setTurnRight(360 - this.getHeading());
//                execute();
//            }
//        } else if (xPosition > wide - xBuffer) {
////            System.out.println("Gefahr");
//            if ((this.getHeading() > 0) && (this.getHeading() < 90)) {
//                setAhead(5);
//                this.setTurnLeft(90 - this.getHeading());
//                execute();
//            } else if ((this.getHeading() > 90) && (this.getHeading() < 180)) {
//                setAhead(5);
//                this.setTurnRight(180 - this.getHeading());
//                execute();
//            }
//        } else {
//            this.setTurnRight(0);
//            this.setTurnLeft(0);
//        }
//
//        execute();
//    }
//
//}
}
