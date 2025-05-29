//#############################################################################################################
//  _______  _        _______    _______  _______  _______           _______     ___  _______  _______ 
// (       )| \    /\(  ____ \  (  ____ \(  ____ \(  ____ )|\     /|(  ___  )   /   )/ ___   )(  ____ \
// | () () ||  \  / /| (    \/  | (    \/| (    \/| (    )|| )   ( || (   ) |  / /) |\/   )  || (    \/
// | || || ||  (_/ / | (_____   | (_____ | (__    | (____)|| |   | || |   | | / (_) (_   /   )| |      
// | |(_)| ||   _ (  (_____  )  (_____  )|  __)   |     __)( (   ) )| |   | |(____   _)_/   / | |      
// | |   | ||  ( \ \       ) |        ) || (      | (\ (    \ \_/ / | |   | |     ) ( /   _/  | |      
// | )   ( ||  /  \ \/\____) |  /\____) || (____/\| ) \ \__  \   /  | (___) |     | |(   (__/\| (____/\
// |/     \||_/    \/\_______)  \_______)(_______/|/   \__/   \_/   (_______)     (_)\_______/(_______/
//                                                                                                    
// Library to control the Makerbase Servo42C driver
//
// The platform.ini file is configured for the
// Seeed Studio XIAO ESP32C3 Tiny MCU
// For a different board please adjust the platform.ini file as needed
//#############################################################################################################


//#############################################################################################################
// How to flash the Seeed Studio XIAO ESP32C3 Tiny MCU?
//
// Connect the board via USB.
//
// Set it into recovery mode or what it is called using this procedure:
//     1. Press the B/Boot button and hold it
//     2. Press the R/Reset button 
//     3. Release the R button first and then release the B button
//     The board should now be in the recovery mode or something. Some pins 
//     go low. if a small LED is connected to a Pin like in the example (D0)
//     the LED will not shine up as it doesn't enter the main loop
//
// On Linux I have to chmod the USB device to write to it..
//    The device ID can change. Sometimes it is ttyACM0 sometimes ttyACM1 
//    Linux command: sudo chmod 777 /dev/ttyACM0 
//    Different system may have different device ids. Vcode can help finding it
//    It scans for the device in the serial monitor and also when it tries to flash
//    and shows the id in the console. Id may change with everytime it is rebooted
//
// Press upload on vstudio to flash the firmware to the board
//
// Press the R button after success and then unplug USB and plug it in again
// this step is needed for me to get something running after flashing
// 
// Done (It should be..)
//
// Pretty complicated and maybe this is just an issue with my system. First time
// flashing worked without all the recovery mode stuff.
//#############################################################################################################


//#############################################################################################################
// Main includes
//#############################################################################################################
#include <Arduino.h>

#include "main.h"
#include "servo42c.h"

SERVO42C *servo_stepper;

HardwareSerial mks_serial(0);



void setup() {
  //pinMode(D0,OUTPUT); // used it for a little led on that pin to make flashing easier. The xiao seeed board needs to be set into recovery mode or whatever. hard to see without visual feedback.
  Serial.begin(9600);
  mks_serial.begin(38400,SERIAL_8N1,RX_PIN,TX_PIN);
  delay(2000);
  // create and configure the 42C servo stepper
  servo_stepper = new SERVO42C();
  servo_stepper->init( mks_serial );
  servo_stepper->set_slave_address( MKS42C_ADDRESS_DEFAULT );  // set the drivers slave address (this needs to be the same as set on the stepper driver itself)
  servo_stepper->set_max_current( MKS42C_MAXCURRENT_DEFAULT ); // set max current
  servo_stepper->set_max_torque( MKS42C_MAXTORQUE_DEFAULT );   // set max torque
  servo_stepper->set_enable_mode( MKS42C_ENABLEMODE_DEFAULT ); // set enable mode to active low (enable pin low = motor enabled )
  servo_stepper->set_subdivision( MKS42C_MICROSTEPS_DEFAULT ); // set microsteps
  servo_stepper->set_subdivision_interpolation( MKS42C_ENABLEMICROSTEPS_DEFAULT ); // enable microstepping I guess
  vTaskDelay(50);
  //servo_stepper->set_move_steps( 0, 80, 6000 ); // dir, speed, steps
  //vTaskDelay(1000); // let it run a little
  //servo_stepper->set_stop_motor(); // enforce motor to stop
}

void loop(){ 

  Serial.println("");
  float aerr = servo_stepper->get_shaft_angle_error();
  int prec = servo_stepper->get_pulses_received();
  int encv = servo_stepper->get_encoder_value();
  Serial.print( "  Shaft error: " );
  Serial.print(aerr);
  Serial.print( "  Pulses: " );
  Serial.print(prec);
  Serial.print( "  Encoder: " );
  Serial.print(encv);
  Serial.print("  ");
  //digitalWrite(D0, HIGH); // make tiny LED go blink
  //vTaskDelay(500);
  //digitalWrite(D0, LOW);
  vTaskDelay(500);

}

