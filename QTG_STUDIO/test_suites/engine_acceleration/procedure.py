"""Test-specific initialization configuration; BRUNNER source remains unchanged."""
TEST_REFERENCE = '1.f.(1)'
USE_SETTLED_REFERENCE_ALTIMETER = True
DEFAULT_CONDITIONS = {
    "Flt cond": "Approach or Landing",
    "Alt[ft]": "2000",
    "IAS[kts]": "80",
    "Flaps": "0",
    "Throttle[%]": "5",
    "Latitude[deg]": "32.224",
    "Longitude[deg]": "74.1095",
    "Heading[deg]": "35",
    "Pitch[deg]": "0",
    "Bank[deg]": "0",
    "Temperature[C]": "15"
}
DEFAULT_DURATION_S = 21

DURATION_SECONDS = 21
DEFAULT_INITIAL_CONDITIONS = {'alt_ft':2000.,'ias_kt':80.,'throttle':5.,'flaps':0.,'lat':32.224,'lon':74.1095,'heading_deg':35.,'pitch_deg':0.,'bank_deg':0.}
