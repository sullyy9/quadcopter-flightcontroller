#!/bin/bash

st-flash erase
st-flash write ./build/release/ryansullivan_quadcopter_flightcontroller.bin 0x08000000
st-flash reset

