package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public class LED extends SubsystemBase{
  public static final int pwmPort = 9;
  private static final int ledLength = 50;
  private final AddressableLED led = new AddressableLED(pwmPort);
  private AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(ledLength);
  private double totalBrightness;
  private final double maxTotalBrightness = 1550;
  private double brightnessDivider;

  public LED() {
    led.setLength(ledLength);
    led.start();
  }

  public void setLedBufferToSolidColor(Color color){
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      ledBuffer.setLED(ledIndex, color);
    }
  }

  public Command setLedsToSolidColor(Color color){
    return runLeds(() -> setLedBufferToSolidColor(color));
  }

  public Command runLeds(Runnable action) {
    return runOnce(
      () -> {
        action.run();
        showLeds();
      })
      .ignoringDisable(true);
  }

  public void showLeds() {
    powerLimitLeds();
    led.setData(ledBuffer);
  }

  public void powerLimitLeds() {
    totalBrightness = 0;
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      totalBrightness += ledBuffer.getLED(ledIndex).red + ledBuffer.getLED(ledIndex).green + ledBuffer.getLED(ledIndex).blue;
    }
    brightnessDivider = totalBrightness/maxTotalBrightness;
    if(maxTotalBrightness>1) {
      for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
        ledBuffer.setLED(ledIndex, new Color(ledBuffer.getLED(ledIndex).red/brightnessDivider, ledBuffer.getLED(ledIndex).green/brightnessDivider, ledBuffer.getLED(ledIndex).blue/brightnessDivider));
      }
    }

  }
}
