### AP20 timestampers

This package consists of two parts:

1. A read timestamp program, which gets the timestamps and puts them in a FIFO pipe. For acces to the Generic Timestamp Engine (GTE), it needs to be run as sudo.
2. A timestamp publishing program, which reads the timestamps from the FIFO pipe, merges them with the appropriate position measurements and then publishes these new position measurements with the new timestamps.