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

  public LED() {
    led.setLength(ledLength);
    led.start();
  }

  public void showLeds() {
    led.setData(ledBuffer);
  }

  public Command runLeds(Runnable action) {
    return runOnce(
      () -> {
        action.run();
        showLeds();
      })
      .ignoringDisable(true);
  }

  public void setLedBufferToSolidColor(Color color){
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      ledBuffer.setLED(ledIndex, color);
    }
  }

  public Command setLedsToSolidColor(Color color){
    return runLeds(() -> setLedBufferToSolidColor(color));
  }

  public void powerLimitLeds() {
    totalBrightness = 0;
    for (int ledIndex = 0; ledIndex < ledLength; ledIndex++) {
      totalBrightness += ledBuffer
    }

  }
}
