# nmea_gps_driver

ROS driver to parse NMEA strings and publish standard ROS GPS message types. Does not require the GPSD daemon to be running.



This package is deprecated and you should use nmea_navsat_driver going forward.

## Use

```shell
rosrun nmea_gps_driver nmea_gps_driver.py _port:=/dev/ttyUSB0 _baud:=9600
```



## API

This package has no released Code API.

The ROS API documentation and other information can be found at http://ros.org/wiki/nmea_gps_driver

## Reference

[1] [UTC/GPS time converter](https://www.gw-openscience.org/gps/)

[2] [leapsecond UTC/TAI/GPS](http://leapsecond.com/java/gpsclock.htm)

[3] [UTC to GPS Time Correction](https://confluence.qps.nl/qinsy/latest/utc-to-gps-time-correction-32245263.html)
