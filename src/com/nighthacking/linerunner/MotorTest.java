package com.nighthacking.linerunner;

import com.pi4j.io.gpio.GpioController;
import com.pi4j.io.gpio.GpioFactory;
import com.pi4j.io.gpio.GpioPinDigitalOutput;
import com.pi4j.io.gpio.Pin;
import com.pi4j.io.gpio.PinState;
import com.pi4j.io.gpio.RaspiPin;
import com.pi4j.wiringpi.Gpio;
import com.pi4j.wiringpi.SoftPwm;

/**
 * @author Stephen Chin <steveonjava@gmail.com>
 */
public class MotorTest {

  private static final int MOTOR1_PWM = 4;
  private static final int MOTOR2_PWM = 3;
  private static final Pin MOTOR1_DIR = RaspiPin.GPIO_02;
  private static final Pin MOTOR2_DIR = RaspiPin.GPIO_05;

  public void motorTest() {
    GpioController gpio = GpioFactory.getInstance();
    Gpio.wiringPiSetup();
    SoftPwm.softPwmCreate(MOTOR1_PWM, 0, 100);
    SoftPwm.softPwmCreate(MOTOR2_PWM, 0, 100);
    GpioPinDigitalOutput motor1Dir = gpio.provisionDigitalOutputPin(MOTOR1_DIR, "Motor1Dir");
    GpioPinDigitalOutput motor2Dir = gpio.provisionDigitalOutputPin(MOTOR2_DIR, "Motor2Dir");
    System.out.println("forward");
    motor1Dir.setState(PinState.HIGH);
    motor2Dir.setState(PinState.LOW);
    SoftPwm.softPwmWrite(MOTOR1_PWM, 100);
    SoftPwm.softPwmWrite(MOTOR2_PWM, 100);
    Gpio.delay(2000);
    System.out.println("back");
    motor1Dir.setState(PinState.LOW);
    motor2Dir.setState(PinState.HIGH);
    SoftPwm.softPwmWrite(MOTOR1_PWM, 100);
    SoftPwm.softPwmWrite(MOTOR2_PWM, 100);
    Gpio.delay(2000);
    System.out.println("spin");
    motor1Dir.setState(PinState.HIGH);
    motor2Dir.setState(PinState.HIGH);
    SoftPwm.softPwmWrite(MOTOR1_PWM, 100);
    SoftPwm.softPwmWrite(MOTOR2_PWM, 100);
    Gpio.delay(2000);
    System.out.println("stop");
    SoftPwm.softPwmWrite(MOTOR1_PWM, 0);
    SoftPwm.softPwmWrite(MOTOR2_PWM, 0);
    gpio.shutdown();
  }
}
