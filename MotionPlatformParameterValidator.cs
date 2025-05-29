using System;

public static class MotionPlatformParameterValidator
{
    public const float MaxRotationDegrees = 15.0f; // ±15°
    public const float MaxTranslationMm = 160.0f;  // ±160 mm
    public const float MaxSpeedDegreesPerSec = 30.0f;
    public const float MaxSpeedMmPerSec = 300.0f;
    public const float MaxFrequencyHz = 5.0f;
    public const float MaxAmplitudeDegrees = 20.0f;
    public const float MaxAmplitudeMm = 80.0f;
    public const float MinPhase = -360.0f;
    public const float MaxPhase = 360.0f;

    public static bool ValidateRotation(float value) => Math.Abs(value) <= MaxRotationDegrees;
    public static bool ValidateTranslation(float value) => Math.Abs(value) <= MaxTranslationMm;
    public static bool ValidateSpeedRotation(float value) => Math.Abs(value) <= MaxSpeedDegreesPerSec;
    public static bool ValidateSpeedTranslation(float value) => Math.Abs(value) <= MaxSpeedMmPerSec;
    public static bool ValidateAmplitudeRotation(float value) => Math.Abs(value) <= MaxAmplitudeDegrees;
    public static bool ValidateAmplitudeTranslation(float value) => Math.Abs(value) <= MaxAmplitudeMm;
    public static bool ValidateFrequency(float value) => value >= 0 && value <= MaxFrequencyHz;
    public static bool ValidatePhase(float value) => value >= MinPhase && value <= MaxPhase;
} 