#!/usr/bin/env python3

from bitbots_cm730.lowlevel.controller.controller import Controller, get_mx28_register_table, get_cm730_register_table
from bitbots_cm730.lowlevel.serial import Serial

ctrl = Controller(Serial('/dev/ttyUSB0'))

cm = get_cm730_register_table()
mx = get_mx28_register_table()

ctrl.write_register(200, cm.dxl_power, 1)

for i in range(1, 200):
    print(i, ctrl.ping(i))

ctrl.write_register(200, cm.dxl_power, 0)
