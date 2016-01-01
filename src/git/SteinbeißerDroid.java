/**
 * Copyright (c) 2001-2014 Mathew A. Nelson and Robocode contributors All rights
 * reserved. This program and the accompanying materials are made available
 * under the terms of the Eclipse Public License v1.0 which accompanies this
 * distribution, and is available at
 * http://robocode.sourceforge.net/license/epl-v10.html
 */
package git;

import java.awt.Color;
import java.util.Hashtable;
import java.awt.geom.Point2D;
import java.util.ArrayList;
import robocode.*;
import robocode.util.Utils;
import robocode.MessageEvent;
import robocode.TeamRobot;

public class SteinbeißerDroid extends TeamRobot implements Droid {

    public Hashtable<String, RobotInfo> robotsList = new Hashtable<String, RobotInfo>();
    RobotInfo robotInformation = new RobotInfo();
    String myNameIs = "test.SteinbeißerDroid*";
    String bigBoss = "test.SteinbeißerLeader*";
    String target;
    double a, b;
    private double oldEnemyHeading;
    public boolean movingForward = true;

    @Override
    public void run() {

        RobotColors c = new RobotColors();
        c.bodyColor = Color.black;
        c.gunColor = Color.green;
        c.bulletColor = Color.green;
        setBodyColor(c.bodyColor);
        setGunColor(c.gunColor);
        setBulletColor(c.bulletColor);

        do {
            aimAndShoot();
            execute();

        } while (true);

    }

    /**
     * onMessageReceived: What to do when our leader sends a message
     */
    @Override
    public void onMessageReceived(MessageEvent e) {

        robotsList = (Hashtable<String, RobotInfo>) e.getMessage();
        robotInformation = robotsList.get(bigBoss);
        target = robotInformation.getTARGET();
//        System.out.println(" Ich bin an der X-Koordinate " + getX() + " Y-Koordinate " + getY());
        if (target == null) {
//            System.out.println("KEIN ZIEL!");
            goTo(robotsList.get(bigBoss));

        } else {
//            System.out.println("HABE ZIEL!");
            goTo(robotsList.get(target));
        }

    }

    private void goTo(Point2D destination) {
        //http://www.jasonsjava.com/?cat=2
//		stop(); // don't move again until we are ready
        Point2D location = new Point2D.Double(getX(), getY());
//        System.out.println("JETZIGE POSITION " + location);
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

//        out.println("Going to: " + destination);
        turnRight(angle); // complete the turn before going any distance (blocking call from Robot class)
        setAhead(distance);

        // must be called because setAhead() will not move without a call to execute().
    }

//    double absoluteBearing(double x1, double y1, double x2, double y2) {
//        double xo = x2 - x1;
//        double yo = y2 - y1;
//        double hyp = Point2D.distance(x1, y1, x2, y2);
//        double arcSin = Math.toDegrees(Math.asin(xo / hyp));
//        double bearing = 0;
//
//        if (xo > 0 && yo > 0) { // both pos: lower-Left
//            bearing = arcSin;
//        } else if (xo < 0 && yo > 0) { // x neg, y pos: lower-right
//            bearing = 360 + arcSin; // arcsin is negative here, actuall 360 - ang
//        } else if (xo > 0 && yo < 0) { // x pos, y neg: upper-left
//            bearing = 180 - arcSin;
//        } else if (xo < 0 && yo < 0) { // both neg: upper-right
//            bearing = 180 - arcSin; // arcsin is negative here, actually 180 + ang
//        }
//
//        return bearing;
//    }

    private void aimAndShoot() { //http://robowiki.net/wiki/Circular_Targeting
        if (target != null) {
            RobotInfo e = robotsList.get(target);
            double bulletPower = getFirepower();//Math.min(3.0, getEnergy());
            double myX = getX();
            double myY = getY();
            double absoluteBearing = getHeadingRadians() + e.getBearingRad();
            double enemyX = getX() + e.distance(getX(), getY()) * Math.sin(absoluteBearing);
            double enemyY = getY() + e.distance(getX(), getY()) * Math.cos(absoluteBearing);
            double enemyHeading = e.getHeadingRad();
            double enemyHeadingChange = enemyHeading - oldEnemyHeading;
            double enemyVelocity = e.getVelocity();
            oldEnemyHeading = enemyHeading;

            double deltaTime = 0;
            double battleFieldHeight = getBattleFieldHeight(),
                    battleFieldWidth = getBattleFieldWidth();
            double predictedX = enemyX, predictedY = enemyY;
            while ((++deltaTime) * (20.0 - 3.0 * bulletPower)
                    < Point2D.Double.distance(myX, myY, predictedX, predictedY)) {
                predictedX += Math.sin(enemyHeading) * enemyVelocity;
                predictedY += Math.cos(enemyHeading) * enemyVelocity;
                enemyHeading += enemyHeadingChange;
                if (predictedX < 18.0
                        || predictedY < 18.0
                        || predictedX > battleFieldWidth - 18.0
                        || predictedY > battleFieldHeight - 18.0) {

                    predictedX = Math.min(Math.max(18.0, predictedX),
                            battleFieldWidth - 18.0);
                    predictedY = Math.min(Math.max(18.0, predictedY),
                            battleFieldHeight - 18.0);
                    break;
                }
            }
            double theta = Utils.normalAbsoluteAngle(Math.atan2(
                    predictedX - getX(), predictedY - getY()));
            a = getX() + 1500 * Math.sin(theta);
            b = getY() + 1500 * Math.cos(theta);
            theta = Utils.normalRelativeAngle(
                    theta - getGunHeadingRadians());
            setTurnGunRightRadians(theta);
            System.out.println(bulletPower);
            fire(bulletPower);
        }
    }

    private double getFirepower() { //errechnet die beste Feuerkraft je nach Entfernung zur target
        //Grenzen sind momentan noch grobe Schätzungen
        //Formel: Schadeneffizienz = E =   (6x-2)*min(1, 18/(d*asin(8/(20-3x)))     /     (10+ceil(2*x)
        //wobei x = Firepower; d = distance
        double power;
        double distance;
        RobotInfo bot = (RobotInfo) robotsList.get(target);
        distance = bot.distance(this.getX(), this.getY());
        if (distance > 300) {
            power = 0.8;
        } else if (distance < 75) {
            power = robocode.Rules.MAX_BULLET_POWER;
        } else {
            power = 1.5;
        }
        return power;
    }

    public Point2D[] generate() {
        double height = this.getBattleFieldHeight();
        double width = this.getBattleFieldWidth();
        Point2D pointArray[];
        ArrayList points = new ArrayList(1);
        double theta;
        double dist;
        double safeDistance = 40;
        for (int i = 0; i < 200; i++) {
            theta = Math.random() * Math.PI * 2.0;
            dist = Math.random() * 100.0 + 200.0;
            Point2D p = projectPoint(new Point2D.Double(this.getX(), this.getY()), theta, dist);
            if (p.getX() > safeDistance && p.getX() < width - 2 * safeDistance && p.getY() > safeDistance && p.getY() < height - 2 * safeDistance) {
                points.add(p);//WARNUNG!!!
            }
        }
        pointArray = new Point2D[points.size()];
        for (int i = 0; i < points.size(); i++) {
            pointArray[i] = (Point2D) points.get(i);
        }
//        for (int i = 0; i< points.size(); i++){
//            System.out.println(a[i]);
//        }
        return pointArray;
    }

    private static Point2D projectPoint(Point2D startPoint, double theta, double dist) { //von Shiz
        return new Point2D.Double(startPoint.getX() + dist * Math.sin(theta), startPoint.getY() + dist * Math.cos(theta));
    }

    @Override
    public void onHitWall(HitWallEvent event) {

        reverseDirection();
//        out.println("Ich habe die Wand getroffen " + event.getBearing() + "180");
//        turnLeft(event.getBearing() + 45);
//        ahead(120);
    }

    @Override
    public void onHitRobot(HitRobotEvent e) {
        if (isTeammate(e.getName())) {
            reverseDirection();
//            System.out.println("Leader getroffen");
        }

    }

    public void reverseDirection() {
        if (movingForward) {
            setMaxVelocity(6);
            setMaxTurnRate(Rules.MAX_TURN_RATE);
            setBack(40000);
            movingForward = false;
        } else {
            setMaxVelocity(6);
            setMaxTurnRate(Rules.MAX_TURN_RATE);
            setAhead(40000);
            movingForward = true;
        }
    }
}
