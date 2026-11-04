"""Test-specific initialization configuration; BRUNNER source remains unchanged."""
TEST_REFERENCE = '1.f.(2)'
USE_SETTLED_REFERENCE_ALTIMETER = True
DEFAULT_CONDITIONS = {
    "Flt cond": "Ground",
    "Alt[ft]": "755",
    "IAS[kts]": "0",
    "Flaps": "0",
    "Throttle[%]": "100",
    "Latitude[deg]": "32.235",
    "Longitude[deg]": "74.135",
    "Temperature[C]": "15"
}
DEFAULT_DURATION_S = 21

DURATION_SECONDS = 21
DEFAULT_INITIAL_CONDITIONS = {'alt_ft':755.,'ias_kt':0.,'throttle':100.,'flaps':0.,'on_ground':True}
