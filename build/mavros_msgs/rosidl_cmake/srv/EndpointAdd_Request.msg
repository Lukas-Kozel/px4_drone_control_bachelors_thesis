#
# Adds endpoint to router
#

uint8 TYPE_FCU = 0
uint8 TYPE_GCS = 1
uint8 TYPE_UAS = 2

string url          # mavconn URL or topic prefix for TYPE_UAS
uint8 type          # should be set to one of the TYPE_xxx
