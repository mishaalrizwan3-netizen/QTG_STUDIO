"""Test-specific initialization configuration; BRUNNER source remains unchanged."""
TEST_REFERENCE = '1.f.(2)'
USE_SETTLED_REFERENCE_ALTIMETER = True
DEFAULT_CONDITIONS = {
    "Flt cond": "Approach or Landing",
    "Alt[ft]": "2000",
    "IAS[kts]": "80",
    "Flaps": "0",
    "Throttle[%]": "100",
    "Latitude[deg]": "32.235",
    "Longitude[deg]": "74.135",
    "Temperature[C]": "15"
}
DEFAULT_DURATION_S = 21

DURATION_SECONDS = 21
DEFAULT_INITIAL_CONDITIONS = {'alt_ft':2000.,'ias_kt':80.,'throttle':100.,'flaps':0.}
