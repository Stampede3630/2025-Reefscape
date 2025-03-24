// Copyright (c) 2025 FRC 3630
// https://github.com/Stampede3630
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.leds;

import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.FieldConstants;
import frc.robot.util.TimedSubsystem;
import java.util.List;
import java.util.Optional;

public class Leds extends TimedSubsystem {
  // Constants
  private static final boolean prideLeds = false;
  private static final int minLoopCycleCount = 10;
  private static final int length = 300;
  private static final Section fullSection = new Section(0, length);
  private static final Section topSection = new Section(length / 2, length);
  private static final Section bottomSection = new Section(0, length / 2);
  private static final Section topThreeQuartSection = new Section(length / 4, length);
  private static final Section bottomQuartSection = new Section(0, length / 4);
  private static final double strobeDuration = 0.1;
  private static final double breathFastDuration = 0.5;
  private static final double breathSlowDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveDisabledCycleLength = 15.0;
  private static final double waveDisabledDuration = 2.0;
  private static final double autoFadeTime = 2.5; // 3s nominal
  private static final double autoFadeMaxTime = 5.0; // Return to normal
  private static final Color l1PriorityColor = Color.kOrangeRed;
  private static final Color l2PriorityColor = Color.kCyan;
  private static final Color l3PriorityColor = Color.kBlue;
  private static final Color l4PriorityColor = Color.kPurple;
  private static Leds instance;
  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;
  private final Notifier loadingNotifier;
  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hpAttentionAlert = false;
  public boolean endgameAlert = false;
  public boolean autoScoringReef = false;
  public boolean autoScoring = false;
  public boolean superstructureCoast = false;
  public boolean superstructureEstopped = false;
  public boolean lowBatteryAlert = false;
  public boolean characterizationMode = false;
  public boolean visionDisconnected = false;
  public boolean climbing = false;
  public boolean coralGrabbed = false;
  public Optional<FieldConstants.ReefLevel> firstPriorityLevel = Optional.empty();
  public Optional<FieldConstants.ReefLevel> secondPriorityLevel = Optional.empty();
  public FieldConstants.ReefLevel autoScoringLevel = FieldConstants.ReefLevel.L4;
  public boolean firstPriorityBlocked = false;
  public boolean secondPriorityBlocked = false;
  public Color hexColor = Color.kBlack;
  public Color secondaryHexColor = Color.kBlack;
  private Optional<Alliance> alliance = Optional.empty();
  private Color disabledColor = Color.kYellow;
  private Color secondaryDisabledColor = Color.kDarkBlue;
  private boolean lastEnabledAuto = false;
  private double lastEnabledTime = 0.0;
  private boolean estopped = false;

  private Leds() {
    super("LEDs");
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
    loadingNotifier =
        new Notifier(
            () -> {
              synchronized (this) {
                breath(
                    fullSection,
                    Color.kWhite,
                    Color.kBlack,
                    breathSlowDuration,
                    System.currentTimeMillis() / 1000.0);
                leds.setData(buffer);
              }
            });
    loadingNotifier.startPeriodic(0.02);
  }

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  public synchronized void timedPeriodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
      disabledColor =
          alliance
              .map(alliance -> alliance == Alliance.Blue ? Color.kBlue : Color.kRed)
              .orElse(disabledColor);
      secondaryDisabledColor = alliance.isPresent() ? Color.kBlack : secondaryDisabledColor;
    }

    // Update auto state
    if (DriverStation.isEnabled()) {
      lastEnabledAuto = DriverStation.isAutonomous();
      lastEnabledTime = Timer.getTimestamp();
    }

    // Update estop state
    if (DriverStation.isEStopped()) {
      estopped = true;
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Stop loading notifier if running
    loadingNotifier.stop();

    // Strategy priorities
    hexColor = renderPriority(firstPriorityLevel, firstPriorityBlocked, topSection);
    secondaryHexColor = renderPriority(secondPriorityLevel, secondPriorityBlocked, bottomSection);

    // Select LED mode
    solid(fullSection, Color.kBlack); // Default to off
    if (estopped) {
      solid(fullSection, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      /* if (lastEnabledAuto && Timer.getTimestamp() - lastEnabledTime < autoFadeMaxTime) {
        // Auto fade
        wave(
            new Section(
                0,
                (int) (length * (1 - ((Timer.getTimestamp() - lastEnabledTime) / autoFadeTime)))),
            Color.kGold,
            Color.kDarkBlue,
            waveFastCycleLength,
            waveFastDuration);
      } */ if (lowBatteryAlert) {
        // Low battery
        solid(fullSection, Color.kBlack);
      } else if (prideLeds) {
        // Pride stripes
        stripes(
            fullSection,
            List.of(
                Color.kBlack,
                Color.kRed,
                Color.kOrangeRed,
                Color.kYellow,
                Color.kGreen,
                Color.kBlue,
                Color.kPurple,
                Color.kBlack,
                new Color(0.15, 0.3, 1.0),
                Color.kDeepPink,
                Color.kWhite,
                Color.kDeepPink,
                new Color(0.15, 0.3, 1.0)),
            3,
            5.0);
      } else {
        // Default pattern
        wave(
            fullSection,
            disabledColor,
            secondaryDisabledColor,
            waveDisabledCycleLength,
            waveDisabledDuration);
      }

      // Vision disconnected alert
      if (visionDisconnected) {
        strobe(bottomQuartSection, Color.kRed, Color.kBlack, strobeDuration);
      }

    } else if (DriverStation.isAutonomous()) {
      if (characterizationMode) {
        strobe(fullSection, Color.kGold, Color.kBlack, 0.5);
      } else {
        solid(fullSection, Color.kAqua);
      }
    } else {
      solid(topSection, hexColor);
      solid(bottomSection, secondaryHexColor);

      // Auto scoring reef
      if (autoScoring) {
        solid(fullSection, Color.kAqua);
        solid(
            bottomQuartSection,
            switch (autoScoringLevel) {
              case L1 -> l1PriorityColor;
              case L2 -> l2PriorityColor;
              case L3 -> l3PriorityColor;
              case L4 -> l4PriorityColor;
            });
      }

      // Auto scoring
      // if (autoScoring) {
      //   solid(fullSection, Color.kAzure);
      //  }

      // Climbing alert
      if (climbing) {
        strobe(fullSection, Color.kGold, Color.kDarkBlue, strobeDuration);
      }

      // Coral grab alert
      if (coralGrabbed) {
        solid(fullSection, Color.kLime);
      }

      // Human player alert
      if (hpAttentionAlert) {
        strobe(fullSection, Color.kWhite, Color.kBlack, strobeDuration);
      }

      // Endgame alert
      if (endgameAlert) {
        strobe(fullSection, Color.kRed, Color.kGold, strobeDuration);
      }
    }

    // Superstructure estop alert
    if (superstructureEstopped) {
      solid(fullSection, Color.kRed);
    }

    // Update dashboard
    SmartDashboard.putString("LEDs/First Priority", hexColor.toHexString());
    SmartDashboard.putString("LEDs/Second Priority", secondaryHexColor.toHexString());

    // Update LEDs
    leds.setData(buffer);
  }

  private Color solid(Section section, Color color) {
    if (color != null) {
      for (int i = section.start(); i < section.end(); i++) {
        buffer.setLED(i, color);
      }
    }
    return color;
  }

  private Color strobe(Section section, Color c1, Color c2, double duration) {
    boolean c1On = ((Timer.getTimestamp() % duration) / duration) > 0.5;
    return solid(section, c1On ? c1 : c2);
  }

  private Color breath(Section section, Color c1, Color c2, double duration, double timestamp) {
    double x = ((timestamp % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    solid(section, color);
    return color;
  }

  private Color breathCalculate(Section section, Color c1, Color c2, double duration) {
    double x = ((Timer.getTimestamp() % duration) / duration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    var color = new Color(red, green, blue);
    return color;
  }

  private Color breath(Section section, Color c1, Color c2, double duration) {
    return breath(section, c1, c2, duration, Timer.getTimestamp());
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      x %= 180.0;
      buffer.setHSV(i, (int) x, 255, 255);
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getTimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = section.end() - 1; i >= section.start(); i--) {
      x += xDiffPerLed;
      double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
      if (Double.isNaN(ratio)) {
        ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
      }
      if (Double.isNaN(ratio)) {
        ratio = 0.5;
      }
      double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
      double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
      double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
      buffer.setLED(i, new Color(red, green, blue));
    }
  }

  private void stripes(Section section, List<Color> colors, int stripeLength, double duration) {
    int offset = (int) (Timer.getTimestamp() % duration / duration * stripeLength * colors.size());
    for (int i = section.end() - 1; i >= section.start(); i--) {
      int colorIndex =
          (int) (Math.floor((double) (i - offset) / stripeLength) + colors.size()) % colors.size();
      colorIndex = colors.size() - 1 - colorIndex;
      buffer.setLED(i, colors.get(colorIndex));
    }
  }

  private Color renderPriority(
      Optional<FieldConstants.ReefLevel> level, Boolean blocked, Section section) {
    Color primaryColor =
        level.isEmpty()
            ? Color.kBlack
            : switch (level.get()) {
              case L1 -> l1PriorityColor;
              case L2 -> l2PriorityColor;
              case L3 -> l3PriorityColor;
              case L4 -> l4PriorityColor;
            };
    if (!blocked) {
      return primaryColor;
    } else {
      return breathCalculate(
          section,
          primaryColor,
          Color.lerpRGB(primaryColor, Color.kBlack, 0.9),
          breathFastDuration);
    }
  }

  private record Section(int start, int end) {}
}
