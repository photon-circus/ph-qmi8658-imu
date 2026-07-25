use ph_qmi8658::{AccelConfig, Config, Error, GyroConfig};

#[test]
fn main_style_config_literal_remains_valid() {
    let config = Config {
        accel: Some(AccelConfig::DEFAULT),
        gyro: Some(GyroConfig::DEFAULT),
    };

    assert_eq!(config, Config::default());
}

fn classify_error(error: Error) -> &'static str {
    match error {
        Error::Bus => "bus",
        Error::NotPresent => "not-present",
        Error::WrongDevice => "wrong-device",
        Error::NotReady => "not-ready",
        Error::InvalidData => "invalid-data",
        Error::Unsupported => "unsupported",
    }
}

#[test]
fn main_style_exhaustive_error_match_remains_valid() {
    assert_eq!(classify_error(Error::NotReady), "not-ready");
}
