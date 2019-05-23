// Default preferences in raw data format for PROGMEM
//
#define defaultprefs_version 190324
const char defprefs_txt[] PROGMEM = R"=====(
// Voorbeeld configuratie
// Vul hier uw WiFi netwerk gegevens in:
//
// WiFi gegevens
wifi_00 = SSID1/PASSWD1                              // Eerste bekende netwerk
wifi_01 = SSID2/PASSWD2                              // Tweede bekende netwerk
//
// Licht niveau
dimlevel = 150                                       // Licht (0..255) op half niveau
//
// DEBUG
debug = 1                                            // DEBUG aan
//
// Server en map voor registratie
host = smallenburg.nl
post = /stoflamp/stofreg.php
//
// Server en map voor download software (update)
upd_server = smallenburg.nl                          // Server met software
upd_map = /stoflamp/download                         // Map op de server
//
)=====" ;
