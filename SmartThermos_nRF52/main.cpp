//
// main.cpp
// Main file
// ----------------------------------
// Developed with embedXcode+
// https://embedXcode.weebly.com
//
// Project 		SmartThermos_nRF52
//
// Created by 	Ryan Livingston, 3/20/19 12:41 PM
//              Ryan Livingston
//
// Copyright 	© Ryan Livingston, 2019
// Licence 		CC = BY SA NC
//
// See 			SmartThermos_nRF52.ino and ReadMe.txt for references
//
// ----------------------------------
// DO NOT EDIT THIS FILE.
// THE SKETCH IS IN SmartThermos_nRF52.ino
// ----------------------------------
//
// Last update: Jun 30, 2017 release 7.5.0

// IDE selection
#if defined(EMBEDXCODE)

// Core library and main()
// Minimal proxy main() to be updated at build
#include "Arduino.h"

int main(void)
{
    setup();

    for (;;)
    {
        loop();
    }

    return 0;
}

// Sketch
#include "SmartThermos_nRF52.ino"


#endif                                                                          // end embedXcode
