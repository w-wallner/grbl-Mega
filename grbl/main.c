/*
  main.c - An embedded CNC Controller with rs274/ngc (g-code) support
  Part of Grbl

  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"


// Declare system global variable structure
system_t sys;
int32_t sys_position[N_AXIS];      // Real-time machine (aka home) position vector in steps.
int32_t sys_probe_position[N_AXIS]; // Last probe position in machine coordinates and steps.
volatile uint8_t sys_probe_state;   // Probing state value.  Used to coordinate the probing cycle with stepper ISR.
volatile uint8_t sys_rt_exec_state;   // Global realtime executor bitflag variable for state management. See EXEC bitmasks.
volatile uint8_t sys_rt_exec_alarm;   // Global realtime executor bitflag variable for setting various alarms.
volatile uint8_t sys_rt_exec_motion_override; // Global realtime executor bitflag variable for motion-based overrides.
volatile uint8_t sys_rt_exec_accessory_override; // Global realtime executor bitflag variable for spindle/coolant overrides.
#ifdef DEBUG
  volatile uint8_t sys_rt_exec_debug;
#endif

// -----------------------------------------------------------------------------
// Macros to configure pin states
// URL: https://www.avrfreaks.net/forum/macros-ddr-and-port-pin-name
// -----------------------------------------------------------------------------
#define configure_as_input(port, pin)             {DDR ## port &= ~(1 << pin);}
#define configure_as_output(port, pin)            {DDR ## port |= (1 << pin);}

#define pullup_on(port, pin)                      {PORT ## port |= (1 << pin);}
#define pullup_off(port, pin)                     {PORT ## port &= ~(1 << pin);}

#define set_high(port, pin)                       {PORT ## port |= (1 << pin);}
#define set_low(port, pin)                        {PORT ## port &= ~(1 << pin);}

#define set_to_input_high_z(port, pin)            {configure_as_input(port, pin); pullup_off(port, pin)}
#define set_to_input_pullup(port, pin)            {configure_as_input(port, pin); pullup_on(port, pin);}

#define set_to_output_low(port, pin)              {configure_as_output(port, pin); set_low(port, pin)}
#define set_to_output_high(port, pin)             {configure_as_output(port, pin); set_high(port, pin)}

// -----------------------------------------------------------------------------

void port_init(void)
{
  // Port A
  set_to_input_pullup (A,0);    // unused
  set_to_output_low   (A,3);    // E1-EN
  set_to_output_low   (A,4);    // E0-EN

  // Port B
  set_to_input_pullup (B,0);    // SS
  set_to_input_pullup (B,1);    // SCK
  set_to_input_pullup (B,2);    // MOSI
  set_to_input_pullup (B,3);    // MISO
  set_to_output_high  (B,7);    // LED

  // Port C
  set_to_output_low   (C,3);    // E0-STEP
  set_to_output_low   (C,4);    // E1-STEP
  set_to_input_pullup (C,5);    // unused
  set_to_input_pullup (C,6);    // unused

  // Port D
  set_to_input_pullup (D,2);    // RX1
  set_to_input_pullup (D,3);    // TX1
  set_to_input_pullup (D,5);    // unused

  // Port E
  set_to_input_pullup (E,2);    // unused
  set_to_input_pullup (E,3);    // unused
  set_to_output_low   (E,4);    // FAN-2
  set_to_input_pullup (E,5);    // HEAT-2
  set_to_input_pullup (E,6);    // unused
  set_to_input_pullup (E,7);    // unused

  // Port F
  set_to_input_pullup (F,0);    // THERM0
  set_to_input_pullup (F,1);    // THERM1
  set_to_input_pullup (F,2);    // THERM2
  set_to_input_pullup (F,3);    // unused
  set_to_input_pullup (F,4);    // unused
  set_to_input_pullup (F,5);    // unused
  set_to_input_pullup (F,6);    // unused
  set_to_input_pullup (F,7);    // THERM3

  // Port G
  set_to_output_high  (G,0);    // X-MS2
  set_to_output_high  (G,1);    // X-MS1
  set_to_output_high  (G,2);    // Y-MS2
  set_to_input_pullup (G,3);    // unused
  set_to_input_pullup (G,4);    // unused
  set_to_output_low   (G,5);    // BED-HEAT

  // Port H
  set_to_input_pullup (H,0);    // RX2
  set_to_input_pullup (H,2);    // unused
  set_to_output_low   (H,3);    // FAN-1
  set_to_output_low   (H,4);    // HEAT-1
  set_to_output_low   (H,5);    // FAN-0
  set_to_input_pullup (H,6);    // unused
  set_to_input_pullup (H,7);    // unused

  // Port J
  set_to_input_pullup (J,0);    // RX3
  set_to_input_pullup (J,1);    // TX3
  set_to_input_pullup (J,2);    // unused
  set_to_output_low   (J,7);    // PS_ON

  // Port K
  set_to_input_pullup (K,0);    // unused
  set_to_input_pullup (K,1);    // unused
  set_to_input_pullup (K,2);    // unused
  set_to_output_low   (K,3);    // E0-MS1
  set_to_output_low   (K,4);    // E0-MS2
  set_to_output_high  (K,5);    // Z-MS2
  set_to_output_high  (K,6);    // Z-MS1
  set_to_output_high  (K,7);    // Y-MS1

  // Port L
  set_to_output_low   (L,3);    // XY-REF-PWM
  set_to_output_low   (L,4);    // Z-REF-PWM
  set_to_output_low   (L,5);    // E-REF-PWM
  set_to_output_low   (L,6);    // E0-DIR
  set_to_output_low   (L,7);    // E1-DIR
}

int main(void)
{
  // Initialize system upon power-up.
  port_init();
  serial_init();   // Setup serial baud rate and interrupts
  settings_init(); // Load Grbl settings from EEPROM
  stepper_init();  // Configure stepper pins and interrupt timers
  system_init();   // Configure pinout pins and pin-change interrupt

  memset(sys_position,0,sizeof(sys_position)); // Clear machine position.
  sei(); // Enable interrupts

  // Initialize system state.
  #ifdef FORCE_INITIALIZATION_ALARM
    // Force Grbl into an ALARM state upon a power-cycle or hard reset.
    sys.state = STATE_ALARM;
  #else
    sys.state = STATE_IDLE;
  #endif
  
  // Check for power-up and set system alarm if homing is enabled to force homing cycle
  // by setting Grbl's alarm state. Alarm locks out all g-code commands, including the
  // startup scripts, but allows access to settings and internal commands. Only a homing
  // cycle '$H' or kill alarm locks '$X' will disable the alarm.
  // NOTE: The startup script will run after successful completion of the homing cycle, but
  // not after disabling the alarm locks. Prevents motion startup blocks from crashing into
  // things uncontrollably. Very bad.
  #ifdef HOMING_INIT_LOCK
    if (bit_istrue(settings.flags,BITFLAG_HOMING_ENABLE)) { sys.state = STATE_ALARM; }
  #endif

  // Grbl initialization loop upon power-up or a system abort. For the latter, all processes
  // will return to this loop to be cleanly re-initialized.
  for(;;) {

    // Reset system variables.
    uint8_t prior_state = sys.state;
    memset(&sys, 0, sizeof(system_t)); // Clear system struct variable.
    sys.state = prior_state;
    sys.f_override = DEFAULT_FEED_OVERRIDE;  // Set to 100%
    sys.r_override = DEFAULT_RAPID_OVERRIDE; // Set to 100%
    sys.spindle_speed_ovr = DEFAULT_SPINDLE_SPEED_OVERRIDE; // Set to 100%
		memset(sys_probe_position,0,sizeof(sys_probe_position)); // Clear probe position.
    sys_probe_state = 0;
    sys_rt_exec_state = 0;
    sys_rt_exec_alarm = 0;
    sys_rt_exec_motion_override = 0;
    sys_rt_exec_accessory_override = 0;

    // Reset Grbl primary systems.
    serial_reset_read_buffer(); // Clear serial read buffer
    gc_init(); // Set g-code parser to default state
    spindle_init();
    coolant_init();
    limits_init();
    probe_init();
    sleep_init();
    plan_reset(); // Clear block buffer and planner variables
    st_reset(); // Clear stepper subsystem variables.

    // Sync cleared gcode and planner positions to current system position.
    plan_sync_position();
    gc_sync_position();

    // Print welcome message. Indicates an initialization has occured at power-up or with a reset.
    report_init_message();

    // Start Grbl main loop. Processes program inputs and executes them.
    protocol_main_loop();

  }
  return 0;   /* Never reached */
}
