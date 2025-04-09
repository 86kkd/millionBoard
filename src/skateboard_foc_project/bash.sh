# 修改主程序中的头文件引用
sed -i 's/#include "angle_sensor.h"/#include "angle_sensor\/angle_sensor.h"/g' main/main.c
sed -i 's/#include "hx711.h"/#include "hx711\/hx711.h"/g' main/main.c
sed -i 's/#include "nfc.h"/#include "nfc\/nfc.h"/g' main/main.c
sed -i 's/#include "gps.h"/#include "gps\/gps.h"/g' main/main.c
