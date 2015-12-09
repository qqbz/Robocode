package git;

import robocode.ScannedRobotEvent;
import robocode.TeamRobot;

import java.awt.*;
import java.awt.geom.Point2D;
import java.io.IOException;
import java.util.Hashtable;
import java.util.Set;
import java.util.logging.Level;
import java.util.logging.Logger;
import robocode.MessageEvent;
import robocode.Rules;
import robocode.util.Utils;

public class SteinbeißerLeader extends TeamRobot {

    Hashtable robots;
    String target;
    boolean soloMode = false;
    Point2D goal;
    int targetingTime;

    /*
    TO DO:
    chooseTarget(): schnelles Ändern der Target verhindern 
    doRadar(): Bereich eingrenzen wenn alle Bots gescannt wurden, Radarlock einführen
    aimAndShoot(): komplett neu
    getFirepower(): Grenzwerte mit Formel festlegen
    */
    
    /*
    Probleme:
    Roboter aus Liste löschen, wenn sie zerstört werden - woher weiß ich, dass ein Roboter zerstört wurde? Zur not mit Timer lösen?
    */
    
    /*
     onScanned: table updaten
     onDeath: broadcasten
     onMessage: in soloMode umschalten - Umsetzung später
    
     normaler Ablauf:
     eigenen Werte updaten
     scannen
     bewegungsziel berechnen
     schussziel berechnen
     bewegen
     zielen
     schießen
    
     einmalige Sachen:
     Farbe                                       -----
     sich zu Liste hinzufügen                    -----
     */
    
    public void run() {
        RobotColors c = new RobotColors(); //setzt alle Farben
        c.bodyColor = Color.black;
        c.gunColor = Color.green;
        c.radarColor = Color.green;
        c.scanColor = Color.green;
        c.bulletColor = Color.green;
        setBodyColor(c.bodyColor);
        setGunColor(c.gunColor);
        setRadarColor(c.radarColor);
        setScanColor(c.scanColor);
        setBulletColor(c.bulletColor); //

        setAdjustRadarForGunTurn(true); //für Radar notwendig
        setAdjustGunForRobotTurn(true); //

        RobotInfo leader = new RobotInfo(); //fügt sich selbst der Liste hinzu
        robots.put(leader.getName(), leader); 
        leader.setIsAlive(true);
        leader.setIsTeammate(true);
        leader.setName("SteinbeißerLeader"); //

        String[] mates = this.getTeammates(); //fügt alle Teammitglieder der Liste hinzu
        for (String m : mates){ //notwendig? werden ja eh beim scannen erfasst
            RobotInfo friend = new RobotInfo();
            robots.put(m, friend);
            friend.isAlive = true;
            friend.ISTEAMMATE = true;
            friend.setName(m);
        } //

        do {
            leader = (RobotInfo) robots.get("SteinbeißerLeader"); //updated die eigenen Werte
            leader.setLocation(this.getX(), this.getY());
            leader.setEnergy(this.getEnergy());
            leader.setHeadingRad(this.getHeading());
            leader.setVelocity(this.getVelocity());
            leader.setHeadingRad(this.getHeadingRadians()); //

            doRadar();
            aimAndShoot();
            chooseTarget();
            
            try { //sendet Liste und das momentane Ziel an den Droid
                broadcastMessage(robots);
                broadcastMessage(target);
            } catch (IOException ex) {
                Logger.getLogger(SteinbeißerLeader.class.getName()).log(Level.SEVERE, null, ex);
            } //
            
            move();
            execute();
            
        } while (true);

    }
    
    public void onScannedRobot(ScannedRobotEvent e) { //updated den Eintrag des gescanten Robots
        //angelehnt an Shiz
        String scanName = e.getName();
        RobotInfo robot = (RobotInfo) robots.get(scanName);
        if (robot == null) {
            robot = new RobotInfo();
            robots.put(scanName, robot);
        }
        robot.energy = e.getEnergy();
        robot.isAlive = true;
        robot.ISTEAMMATE = isTeammate(scanName);
        robot.velocity = e.getVelocity();
        RobotInfo leader = (RobotInfo)robots.get("SteinbeißerLeader");
        robot.setLocation(projectPoint(new Point2D.Double(leader.getX(), leader.getY()), getHeadingRadians() + e.getBearingRadians(), e.getDistance()));
    }

    public void onMessageReceived(MessageEvent e) { //schaltet in den soloMode wenn der Droid seinen Tod meldet oder updated die Position des Droid
        RobotInfo droid = (RobotInfo) robots.get("SteinbeißerDroid");
        if (e.getMessage() == "I'm dead!") {
            soloMode = true;
            droid.setIsAlive(false);
        }
        if (e.getMessage() instanceof Point2D) {
            droid.setLocation((Point2D) e.getMessage());
        }
    }

    private void chooseTarget() { //wählt nähesten Gegner wenn es momentan kein Ziel gibt; 
        //wechselt Ziel wenn ein Gegner dem Leader um 100 Einheiten näher ist als das momentan Ziel entfern ist
        //TO DO: zu schnelles Wechseln verhindern mit this.getTime()
        String closest = null;
        Point2D myPosition = new Point2D.Double(this.getY(), this.getX());
        RobotInfo bot = (RobotInfo) robots.get(target);
        double targetDistance = bot.distance(myPosition);
        double minDistance = Double.POSITIVE_INFINITY;
        double botDistance;
        Set keys = robots.keySet();
        for (Object key : keys) {
            bot = (RobotInfo) robots.get(key);
            if (!bot.ISTEAMMATE) {
                botDistance = bot.distance(myPosition);
                if (botDistance < minDistance) {
                    minDistance = botDistance;
                    closest = bot.getName();
                }
            }
        }
        bot = (RobotInfo) robots.get(target);
        if (target == null || !bot.isAlive || minDistance < targetDistance-100) {
            this.target = closest;
        }
    }

    private void doRadar() { //zunächst nur spinning radar
        //http://robowiki.net/wiki/One_on_One_Radar#Spinning_radar
        //TO DO: Scanbereich möglichst eingrenzen, Radarlock einbauen
        if (!this.soloMode) {
            turnRadarRightRadians(Double.POSITIVE_INFINITY);
        } else {
             
        }

    }

    private void aimAndShoot() { //TO DO: komplett neu machen
        RobotInfo bot = (RobotInfo) robots.get(target);
        final double FIREPOWER = getFirepower();
        final double ROBOT_WIDTH = 16, ROBOT_HEIGHT = 16;
        // Variables prefixed with e- refer to enemy, b- refer to bullet and r- refer to robot
        final double eAbsBearing = getHeadingRadians() + bot.getBearingRad();
        final double rX = getX(), rY = getY(),
                bV = Rules.getBulletSpeed(FIREPOWER);
        final double eX = rX + bot.distance(this.getX(), this.getY()) * Math.sin(eAbsBearing),
                eY = rY + bot.distance(this.getX(), this.getY()) * Math.cos(eAbsBearing),
                eV = bot.getVelocity(),
                eHd = bot.getHeadingRad();
        // These constants make calculating the quadratic coefficients below easier
        final double A = (eX - rX) / bV;
        final double B = eV / bV * Math.sin(eHd);
        final double C = (eY - rY) / bV;
        final double D = eV / bV * Math.cos(eHd);
        // Quadratic coefficients: a*(1/t)^2 + b*(1/t) + c = 0
        final double a = A * A + C * C;
        final double b = 2 * (A * B + C * D);
        final double c = (B * B + D * D - 1);
        final double discrim = b * b - 4 * a * c;
        if (discrim >= 0) {
            // Reciprocal of quadratic formula
            final double t1 = 2 * a / (-b - Math.sqrt(discrim));
            final double t2 = 2 * a / (-b + Math.sqrt(discrim));
            final double t = Math.min(t1, t2) >= 0 ? Math.min(t1, t2) : Math.max(t1, t2);
            // Assume enemy stops at walls
            final double endX = limit(
                    eX + eV * t * Math.sin(eHd),
                    ROBOT_WIDTH / 2, getBattleFieldWidth() - ROBOT_WIDTH / 2);
            final double endY = limit(
                    eY + eV * t * Math.cos(eHd),
                    ROBOT_HEIGHT / 2, getBattleFieldHeight() - ROBOT_HEIGHT / 2);
            setTurnGunRightRadians(robocode.util.Utils.normalRelativeAngle(
                    Math.atan2(endX - rX, endY - rY)
                    - getGunHeadingRadians()));
            setFire(FIREPOWER);
        }
    }

    private double getFirepower() { //errechnet die beste Feuerkraft je nach Entfernung zur target
        //Grenzen sind momentan noch grobe Schätzungen
        //Formel: Schadeneffizienz = E =   (6x-2)*min(1, 18/(d*asin(8/(20-3x)))     /     (10+ceil(2*x)
        //wobei x = Firepower; d = distance
        double power;
        double distance;
        RobotInfo bot = (RobotInfo) robots.get(target);
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

    private double limit(double value, double min, double max) { //für aimAndShoot
        return Math.min(max, Math.max(min, value));
    }

    private void move() { //Anti-Gravity Movement
        //zugriff auf hashtable: http://www.java2novice.com/java-collections-and-util/hashtable/iterate/
        //fast 1 zu 1 von: http://robowiki.net/wiki/Anti-Gravity_Tutorial
        double xForce = 0, yForce = 0;
        Set keys = robots.keySet();
        for (Object key : keys) {
            RobotInfo robot = (RobotInfo)robots.get(key);
            if (!robot.ISTEAMMATE) {
                double absBearing = Utils.normalAbsoluteAngle(Math.atan2(robot.getX() - this.getX(), robot.getY() - this.getY()));
                double distance = robot.distance(getX(), getY());
                xForce -= Math.sin(absBearing) / (distance * distance);
                yForce -= Math.cos(absBearing) / (distance * distance);
            }
        }
        double angle = Math.atan2(xForce, yForce);
        if (xForce == 0 && yForce == 0) {
            setAhead(5.0); //kleiner Wert damit wir nicht stehen
        } else if (Math.abs(angle - getHeadingRadians()) < Math.PI / 2) {
            setTurnRightRadians(Utils.normalRelativeAngle(angle - getHeadingRadians()));
            setAhead(Double.POSITIVE_INFINITY);
        } else {
            setTurnRightRadians(Utils.normalRelativeAngle(angle + Math.PI - getHeadingRadians()));
            setAhead(Double.NEGATIVE_INFINITY);
        }
    }

    public void onDeath() { //sendet dem Droid seinen Tod und eine letzte Info über alle Roboter
        RobotInfo leader = (RobotInfo) robots.get("SteinbeißerLeader");
        leader.setIsAlive(false);
        try {
            broadcastMessage("I'm dead!");
            broadcastMessage(robots);
        } catch (IOException ex) {
            Logger.getLogger(SteinbeißerLeader.class.getName()).log(Level.SEVERE, null, ex);
        }
    }

    private static Point2D projectPoint(Point2D startPoint, double theta, double dist) { //von Shiz
        return new Point2D.Double(startPoint.getX() + dist * Math.sin(theta), startPoint.getY() + dist * Math.cos(theta));
    }

}
