//! Operating mode state machine for the QMI8658.

/// Operating modes from Table 34 / Figure 11 of the datasheet.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum OperatingMode {
    /// Software reset asserted.
    SoftwareReset,
    /// No power (VDDIO/VDD low).
    NoPower,
    /// Power-on default (all sensors off, normal clock).
    PowerOnDefault,
    /// Power-down (all sensors off, clock off).
    PowerDown,
    /// Magnetometer only.
    MagOnly,
    /// Accelerometer + magnetometer.
    AccelMag,
    /// Accelerometer + gyroscope + magnetometer (9DOF).
    AccelGyroMag,
    /// OIS mode (Accel + Gyro, aODR=0, gODR=0).
    Ois,
    /// Accelerometer + gyroscope only.
    AccelGyroOnly,
    /// Gyroscope only.
    GyroOnly,
    /// Accelerometer only.
    AccelOnly,
    /// Low power accelerometer only.
    LowPowerAccel,
    /// Low power, all sensors off (slow clock).
    LowPowerAllOff,
    /// Wake on Motion (WoM).
    WakeOnMotion,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum TransitionDelay {
    Immediate,
    T0,
    T1PlusT5,
    T2PlusT5,
    T3PlusT5,
    T4PlusT5,
    T6,
    T7,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct ModeTransition {
    pub from: OperatingMode,
    pub to: OperatingMode,
    pub delay: TransitionDelay,
}

const MODE_TRANSITIONS: &[ModeTransition] = &[
    ModeTransition {
        from: OperatingMode::SoftwareReset,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T0,
    },
    ModeTransition {
        from: OperatingMode::NoPower,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T0,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::PowerDown,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::PowerDown,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T7,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::MagOnly,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::MagOnly,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T3PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::AccelMag,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::AccelMag,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T1PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::AccelGyroMag,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::AccelGyroMag,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T1PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::Ois,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::Ois,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T1PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::AccelGyroOnly,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::AccelGyroOnly,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T1PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::GyroOnly,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::GyroOnly,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T4PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::AccelOnly,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::AccelOnly,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T1PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::LowPowerAccel,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::LowPowerAccel,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T2PlusT5,
    },
    ModeTransition {
        from: OperatingMode::PowerOnDefault,
        to: OperatingMode::LowPowerAllOff,
        delay: TransitionDelay::T7,
    },
    ModeTransition {
        from: OperatingMode::LowPowerAllOff,
        to: OperatingMode::PowerOnDefault,
        delay: TransitionDelay::T6,
    },
    ModeTransition {
        from: OperatingMode::LowPowerAllOff,
        to: OperatingMode::WakeOnMotion,
        delay: TransitionDelay::T2PlusT5,
    },
    ModeTransition {
        from: OperatingMode::WakeOnMotion,
        to: OperatingMode::LowPowerAllOff,
        delay: TransitionDelay::T2PlusT5,
    },
    ModeTransition {
        from: OperatingMode::LowPowerAccel,
        to: OperatingMode::LowPowerAllOff,
        delay: TransitionDelay::T2PlusT5,
    },
];

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct OperatingModeStateMachine {
    mode: OperatingMode,
}

impl OperatingModeStateMachine {
    pub const fn new(mode: OperatingMode) -> Self {
        Self { mode }
    }

    pub const fn mode(self) -> OperatingMode {
        self.mode
    }

    pub fn set_mode(&mut self, mode: OperatingMode) {
        self.mode = mode;
    }

    #[allow(dead_code)]
    pub fn transition_to(&mut self, target: OperatingMode) -> Result<TransitionDelay, ()> {
        if self.mode == target {
            return Ok(TransitionDelay::Immediate);
        }

        let delay = transition_delay(self.mode, target).ok_or(())?;
        self.mode = target;
        Ok(delay)
    }
}

pub(crate) fn transition_delay(from: OperatingMode, to: OperatingMode) -> Option<TransitionDelay> {
    if from == to {
        return Some(TransitionDelay::Immediate);
    }

    if matches!(to, OperatingMode::SoftwareReset | OperatingMode::NoPower) {
        return Some(TransitionDelay::Immediate);
    }

    MODE_TRANSITIONS
        .iter()
        .find(|entry| entry.from == from && entry.to == to)
        .map(|entry| entry.delay)
}
