package com.nighthacking.linerunner;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalInput;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.io.serial.Serial;
import com.pi4j.io.serial.SerialDataEvent;
import com.pi4j.io.serial.SerialDataListener;
import com.pi4j.io.serial.SerialFactory;
import com.pi4j.wiringpi.Gpio;
import com.pi4j.wiringpi.SoftPwm;
import java.io.IOException;

/**
 * @author Stephen Chin <steveonjava@gmail.com>
 */
public class LineRunner {

  // Motor 1: dir=10 / pulse=12
  // Motor 2: dir=13 / pulse=7
  // Port 3: GPIO 6 / 14
  // Port 4: GPIO 1
  // Port 5: Serial
  // Port 6: GPIO 4
  // Port 7: GPIO 23 / 22
  // Port 8: GPIO 21 / 11
  // Motor 9: dir=2 / pulse=3
  // Motor 10: dir=5 / pulse=4
  private static final int MOTOR1_PWM = 3;
  private static final int MOTOR2_PWM = 4;
  private static final Pin MOTOR1_DIR = RaspiPin.GPIO_02;
  private static final Pin MOTOR2_DIR = RaspiPin.GPIO_05;
  private static final Pin LINE_FOLLOW_A = RaspiPin.GPIO_06;
  private static final Pin LINE_FOLLOW_B = RaspiPin.GPIO_14;
  private static final int ULTRASONIC = 1;

  public static enum Drive {
    reverse(PinState.HIGH, PinState.LOW),
    stop(PinState.LOW, PinState.LOW),
    forward(PinState.LOW, PinState.HIGH),
    left(PinState.LOW, PinState.LOW),
    right(PinState.HIGH, PinState.HIGH);
    private final PinState motor1;
    private final PinState motor2;
    
    Drive(PinState motor1, PinState motor2) {
      this.motor1 = motor1;
      this.motor2 = motor2;
    }
  };

  public static enum Speed {
    slow(55), medium(65), fast(80), accelerate(100);
    private final int pulse;

    Speed(int pulse) {
      this.pulse = pulse;
    }

    public Speed speedup() {
      return this == accelerate ? accelerate : values()[ordinal() + 1];
    }

    public Speed slowdown() {
      return this == slow ? slow : values()[ordinal() - 1];
    }
  };
  private final GpioController gpio;
  private final GpioPinDigitalOutput motor1Dir;
  private final GpioPinDigitalOutput motor2Dir;
  private final GpioPinDigitalInput lineFollowA;
  private final GpioPinDigitalInput lineFollowB;
  private final Serial remote;
  private volatile Drive drive = Drive.stop;
  private volatile Speed speed = Speed.medium;
  private volatile boolean followLine;
  private volatile Drive lineLocation = Drive.forward;

  public static void main(String[] args) throws IOException, InterruptedException {
    LineRunner lineFollower = new LineRunner();
    lineFollower.run();
  }

  public LineRunner() throws IOException {
    gpio = GpioFactory.getInstance();
    motor1Dir = gpio.provisionDigitalOutputPin(MOTOR1_DIR, "Motor1Dir");
    motor2Dir = gpio.provisionDigitalOutputPin(MOTOR2_DIR, "Motor2Dir");
    lineFollowA = gpio.provisionDigitalInputPin(LINE_FOLLOW_A, "LineFollowA");
    lineFollowB = gpio.provisionDigitalInputPin(LINE_FOLLOW_B, "LineFollowA");
    SoftPwm.softPwmCreate(MOTOR1_PWM, 0, 100);
    SoftPwm.softPwmCreate(MOTOR2_PWM, 0, 100);
    remote = SerialFactory.createInstance();
    remoteListenerSetup();
  }

  private int measureDistance() {
    Gpio.pinMode(ULTRASONIC, Gpio.OUTPUT);
    Gpio.digitalWrite(ULTRASONIC, 0);
    Gpio.delayMicroseconds(2);
    Gpio.digitalWrite(ULTRASONIC, 1);
    Gpio.delayMicroseconds(10);
    Gpio.digitalWrite(ULTRASONIC, 0);
    Gpio.pinMode(ULTRASONIC, Gpio.INPUT);
    long start = System.nanoTime();
    while (Gpio.digitalRead(ULTRASONIC) == 0 && System.nanoTime() - start < 23000000) {
    }
    long mid = System.nanoTime();
    if (mid >= 23000000)
      return -1;  // obstacle too close to detect distance
    while (Gpio.digitalRead(ULTRASONIC) == 1 && System.nanoTime() - mid < 23000000) {
    }
    long end = System.nanoTime();
    return (int) (end - mid) / 58000;
  }

  private void remoteListenerSetup() {
    remote.addListener((SerialDataListener) (SerialDataEvent event) -> {
      String data = event.getData();
      byte command = (byte) data.charAt(2);
      switch (command) {
        case 69: // power/A
          doDrive(Drive.stop);
          gpio.shutdown();
          System.exit(0);
          break;
        case 21: // play/center
          doDrive(drive == Drive.forward ? Drive.stop : Drive.forward);
          break;
        case 7: // rewind/left
          doDrive(drive == Drive.left ? Drive.stop : Drive.left);
          break;
        case 9: // fast forward/right
          doDrive(drive == Drive.right ? Drive.stop : Drive.right);
          break;
        case 64: // plus/up
          changeSpeed(speed.speedup());
          break;
        case 25: // minus/down
          changeSpeed(speed.slowdown());
          break;
        case 67: // return/E
          doDrive(drive == Drive.reverse ? Drive.stop : Drive.reverse);
          break;
        case 68: // test/D
          if (!followLine) { // initiate line following
            followLine = true;
            lineLocation = Drive.forward;
          } else { // terminate line following
            followLine = false;
            Gpio.delay(100);
            doDrive(Drive.stop);
          }
          break;
        default:
          System.out.println("Unrecognized Command: " + command);
      }
    });
    remote.open(Serial.DEFAULT_COM_PORT, 9600);
  }

  private void changeSpeed(Speed speed) {
    this.speed = speed;
    doDrive(drive);
  }

  private void doDrive(Drive drive) {
    motor1Dir.setState(drive.motor1);
    motor1Dir.setState(drive.motor2);
    SoftPwm.softPwmWrite(MOTOR1_PWM, drive == Drive.stop ? 0 : speed.pulse);
    SoftPwm.softPwmWrite(MOTOR2_PWM, drive == Drive.stop ? 0 : speed.pulse);
    this.drive = drive;
  }

  public void run() {
    for (;;) {
      if (followLine) {
        if (measureDistance() < 20) {
          doDrive(Drive.stop);
        } else {
          boolean leftSensor = lineFollowA.getState().isHigh();
          boolean rightSensor = lineFollowB.getState().isHigh();
          if (leftSensor && rightSensor) { // we are lost
            doDrive(lineLocation);
          } else if (!leftSensor && !rightSensor) { // on the line
            doDrive(Drive.forward);
            lineLocation = Drive.forward;
          } else if (!leftSensor && rightSensor) { // slipping off the right
            lineLocation = Drive.left;
            doDrive(Drive.forward);
          } else if (leftSensor && !rightSensor) { // slipping off the left
            lineLocation = Drive.right;
            doDrive(Drive.forward);
          }
        }
        Gpio.delay(1);
      } else {
        Gpio.delay(1000);
      }
    }
  }

}
