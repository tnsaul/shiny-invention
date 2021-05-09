# shiny-invention
Read from a Plantower PMS5003 particulate matter sensor using a Wemos D1 Mini.






## Note: ESP Home Sensor "name" variable suggested fix:

### Configuration variables:

- **name** (**Required**, string): The name for the sensor.  Caution in setting this name as it forms the basis for the unique id for the sensor item. It should only be constructed of the following characters "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_" and the space character.  Any other characters are deleted, thus the following names are identical and will cause issues in Home Assistant - "Sensor PM25" equals "Sensor PM2.5".


Was in this area in "esphome/core/component.cpp":
Line 180:   this->object_id_ = sanitize_string_allowlist(to_lowercase_underscore  (this->name_), HOSTNAME_CHARACTER_ALLOWLIST);

Referencing in "esphome/core/helpers.cpp":
Line 157: const char *HOSTNAME_CHARACTER_ALLOWLIST = "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_";