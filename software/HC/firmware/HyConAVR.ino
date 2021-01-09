/*
    Simple hybrid interface to the Analogparadigm Model-1 analog computer. 
    The same holds true for the two external halt conditions (overflow and the EXT-HALT-input).
    Be careful not to issue c/C (set OP/IC time) or d/D (switch digital output on or off) commands during a single or repetitive run as these commands
   use blocking IO to wait for their respective argument!

   04-AUG-2016  B. Ulmann   Begin implementation
   05-AUG-2016  B. Ulmann   Single/repetitive run implemented
   06-AUG-2016  B. Ulmann   Code-Cleanup, improved timing accuracy, added "t"-command
   07-AUG-2016  B. Ulmann   Changes to match the new Perl-module HyCon.pm
   01-AUG-2016  B. Ulmann   Added basic support for digital potentiometers
   03-SEP-2016  B. Ulmann   Digital potentiometers are now fully supported
   12-MAY-2017  B. Ulmann   Start of 2560-version for the new hybrid controller (lots of new features)
                            New: 'S' -> Enter POTSET-mode
                                 'g<4 hex digits>' -> set address bus and return element ID and readout value
   13-MAY-2017  B. Ulmann   Changed baudrate to 250000, some communication protocol changes
   14-MAY-2017  B. Ulmann   Minor bug fixes
   16-MAY-2017  B. Ulmann   single_run with synchronous mode, POTSET behaviour changed
   22-AUG-2018  B. Ulmann   Start of the port to the new HC module, got rid of the slow digitalWrite calls etc.
   23-AUG-2018  B. Ulmann   Implemented the DPT module routines, only the builtin DPTs are supported right now
   31-AUG-2018  B. Ulmann   Started ADC support
   21-FEB-2019  B. Ulmann   Changed baud rate to 115200 since at least some LINUX systems can't cope with 250000
   05-SEP-2019  B. Ulmann   Added group-commands G, f
   21-SEP-2019  B. Ulmann   Modified group-readout to minimize skew, added address search in setup, baudrate back to 250000
   02-DEC-2019  B. Ulmann   Started adding support for DPT24
   06-DEC-2019  B. Ulmann   Continued adding DPT24 support
   13-DEC-2019  B. Ulmann   Added support for the prototype XBAR-module
   17-DEC-2019  B. Ulmann   Added logging functionality
   18-DEC-2019  B. Ulmann   Single-run-OP-time is now interrupt controlled to minimize timing jitter...
                            Known bug: Data logging logs too few data points when the compute intervals get small.
                            At several seconds everything is as expected, at several ms, things get messy which is
                            not (!) causes by the MIN_SAMPLE_INTERVAL value (a value too small causes the interrupt
                            routine to not finish before it is called the next time).
   23-DEC-2019  B. Ulmann   The data logging got out of sequence after the first tuple, causing a glitch at the start of gnuplot output.
   01-MAR-2020  B. Ulmann   readout is now more precise.
   09-MAR-2020  B. Ulmann   Baud rate set to 250000.
   04-APR-2020  B. Ulmann   ADC read routine modified, it now performs two reads with arithmetic mean which smoothes the result considerably
   05-APR-2020  B. Ulmann   ADC read routine is now about 4 times fast! Who knew that shifts on 32 bit integers were so slow?
   11-APR-2020  B. Ulmann   Oh dear, ADC_READ_DELAY was too short which caused the ADC to lose the MSB, but this only happened
                            due to an erroneous configration procedure... Added locate command 'L' to simplify locating components in
                            larger setups.
   12-APR-2020  B. Ulmann   locate() again.
   26-APR-2020  B. Ulmann   xbar configuration now expects 40 hex nibbles for the production XBAR card with two AD8113.
   27-APR-2020  B. Ulmann   PS-card got an ECO changing its address from $000x to $00Fx to avoid collision with the card in slot 0.
   17-MAY-2020  B. Ulmann   USB interface set to 1 MBit/s - and back, as not being reliable on the Mac.
   08-DEC-2020  B. Ulmann   Rewrote read_adc(...) using Karl-Heinz SPI-implementation as the basis, added init_adc(...).
   09-DEC-2020  B. Ulmann   Some minor cleanups and added comments.
   15-DEC-2020  B. Ulmann   E and F with OVL-Halt enabled and a previous overload only entered OP every second call! Fixed...
*/

/* Port mapping:

    Description   Port    Direction Implemented Tested
    A0-A7         PA0-7   Output    *           *
    A8-A15        PH0-7   Output    *           *
    D0-D7         PK0-7   In/Out    *           *
    Digital IN    PL0-7   Input     *           *
    Digital OUT   PC0-7   Output    *           *
    ModeOP        PE3     Output    *           *
    ModeIC        PE4     Output    *           *
    EXTHALT       PE5     Input     *           *
    /Potset       PE6     Output    *
    ERR           PE7     Output    *           *
    /CS_ADC       PG1     Output    *           *
    /Read         PG2     Output    *           *
    /Write        PG3     Output    *
    ADC_CNV       PG4     Output    *           *
    /Overload     PG5     Input     *           *
    PB1           SCLK    Output    *           *
    PB2           MOSI    Output    *           *
    PB3           MISO    Input     *           *
    SYNC          PB6     Output    *           *

   Spare ports:
    PF0-7, PJ0-7, PD2 / PD3 (second UART TX/RX), PB0/SS (Slave select, unused by SPI, should only be used with caution - may have side effects),
    PB4, PB5, PB7, PD4, PD5, PD6, PD7
*/

#undef DEBUG

#define VERSION "v2.1"
#define VERSION_DATE "20201215"

#include <SPI.h>
#include <TimerThree.h>
#include <TimerFive.h>

#ifndef FALSE
# define FALSE 0
# define TRUE !FALSE
#endif

#define PORTC_DEFAULT B00000000   // Default state of PORTC (digital outputs on front panel)
#define PORTE_DEFAULT B11011111   // Default state of PORTE (control pins etc.)
#define PORTG_DEFAULT B00001110   // /CS_ADC = 1, /READ = 1, /WRITE = 1, CNV = 0

// Some useful global states of the analog computer:
#define PORTE_IC      B11001111   // MIC = 0, MOP = 1, PS = 1, ERR = 1
#define PORTE_OP      B11010111   // MIC = 1, MOP = 0, PS = 1, ERR = 1
#define PORTE_HALT    B11011111   // MIC = 1, MOP = 1, PS = 1, ERR = 1
#define PORTE_POTSET  B10011111   // MIC = 1, MOP = 1, PS = 0, ERR = 1

#define OVL_PORT      PING        // Port to which the overload bus line is mapped
#define OVL_BIT       B00100000   // Number of the bit, the overload bus line is mapped to

#define EXT_HALT_PORT PINE        // Port to which the external halt input is mapped
#define EXT_HALT_BIT  B00100000   // Respective bit on that particular port

#define RW_PORT       PORTG       // Caution: The R/W-bits of port G are addressed in a hardcoded fashion in assert/deassert_read/write()!
#define SYNC_PORT     PORTB       // Caution: The SYNC-bit handling is hardcoded in data2potentiometers()!

#define D_OUT   PORTC             // Port used for the front panel digital outputs
#define D_IN    PINL              // Port used for the front panel digital inputs
#define DBUS_IN PINK              // Port used to read from the data bus of the analog computer

#define RAW     0                 // Moded of operation for data2potentiometers(...)
#define COOKED  1

// Various default values:
#define BAUDRATE        115200                  // Baud rate, can be as high as 2 MBit/s but some LINUX distributions don't support this
#define MAX_GROUP_SIZE  100                     // Maximum number of addresses in a readout group
#define TIMEOUT         10000                   // Timeout in ms for the serial line when reading an integer
#define TIME_DIGITS     6                       // All times (IC/OP) will be specified with six digits (leading zeros)
#define PT_HC           8                       // We have eight builtin digital potentiometers in the hybrid controller
#define PT_DPT24        24                      // ...and 24 DPTs on each DPT24 module
#define PT_VAL_DIGITS   4                       // Digits to specify a potentiometer's wiper setting
#define PT_VAL_MASK     0x03ff                  // The AD5293 digital potentiometers require 10 bit for a wiper position
#define MAX_DPT_MODULES 16                      // A system may accomodate (arbitrarily chosen) up to 16 modules containing digital potentiometers
#define INPUT_BUFF_LEN  MAX_GROUP_SIZE * 5 + 4  // Length of the input buffer for all numeric values
#define READ_DELAY      4                       // Delay for readout of module ids (in microseconds)
#define PS_READ_DELAY   1000                    // A long delay prior to reading the machine units to make sure everything is a stable as can be
#define MAX_SAMPLES     1152                    // Number of 16 bit values usable for logging purposes
#define CONTINGENCY_SAMPLES 128                 // This is subtracted from MAX_SAMPLES to avoid problems due to small timer inaccuracies
#define MIN_SAMPLE_INT  50                      // Minimum duration of a sample interval in microseconds
#define XBAR_CONF_BYTES 20                      // Number of bytes required to configure an XBAR-module - the prototype features one XBAR-chip, thus 
                                                // 80 bits, the production module will have two of these chips with 160 bits of configuration data
#define MODULE_ID_HC    0x0008
#define MODULE_ID_DPT24 0x0009

#define M_UNIT_POS_ADDR 0x00F0                  // Address to read out the positive machine unit
#define M_UNIT_NEG_ADDR 0x00F1                  // Address to read out the negative machine unit
#define M_UNIT_POS_ADDR_OLD 0x0000              // Old addresses to remain compatible with older customer machines - for positive
#define M_UNIT_NEG_ADDR_OLD 0x0001              // and negative reference voltages

// Operating modes of the analog computer
#define MODE_IC     0
#define MODE_OP     1
#define MODE_HALT   2
#define MODE_POTSET 3

// Simple state machine for single/repetitive operation
#define STATE_NORMAL        0
#define STATE_SINGLE_RUN_IC 1
#define STATE_SINGLE_RUN_OP 2
#define STATE_REPETITIVE_IC 3
#define STATE_REPETITIVE_OP 4

// Modes for interrupt control
#define MODE_INACTIVE   -1
#define MODE_ARMED      -2

#define SINGLE_RUN      0
#define REPETITIVE_RUN  1

//***************************************************************************************
// Global variables
//***************************************************************************************
struct DPT_MODULE {  // Each module containing DPTs is associated with one of these structures.
  unsigned int number_of_potentiometers, address, module_id, values[PT_DPT24];
};

int mode,                                           // Current mode of operation of the analog computer
    sync_mode,                                      // If set, single_run (F) will generate a completion message
    module_id;                                      // Remember the ID of the module last read by the ADC
unsigned long int op_start = - 1,                   // Start of OP-mode in microseconds
                  op_end = -1;                      // End of last OP-mode in microseconds
volatile unsigned int ro_group[MAX_GROUP_SIZE],     // Address list for readout group
         ro_group_size = 0;                         // Number of entries in the readout group
int16_t machine_unit_pos, machine_unit_neg;         // Remember the machine units for later conversion
volatile int16_t no_of_samples = 0,                 // Index of last sample, volatile is required due to the interrupt routine
                 samples[MAX_SAMPLES],              // Array for data logging
                 state = STATE_NORMAL;              // The state machine is setup for external control
struct DPT_MODULE dpt_modules[MAX_DPT_MODULES];     // Each module containins DPTs is represented by one entry of this array
SPISettings adc_spi(10000000, MSBFIRST, SPI_MODE3); // Set SPI-mode and speed for accessing the ADC

//***************************************************************************************
//  setup() is called once during startup of the HC module and initializes the various
// components of the system:
//
// - Set the baudrate of the USB interface.
// - Scan the bus for modules such as the HC and DPT24 containing digital potentiometers.
//   Each such interface is associated with a global data structure holding the current
//   configuration bits for the digital potentiometers of this module.
// - Send the Soft Span configuration word to the ADC on the HC board which is done by
//   calling init_adc().
// - Scan the bus for the PS module (this is searched for at the old standard addresses
//   0x0000/0x0001 as well as at the new address 0x00f0/0x00f1 - the new address is used
//   in all systems with a SYSBUS v. 1.4 while previous systems use the old addresses).
//   The two machine units are then read out and the associated 16 bit values stored in 
//   two global variables machine_unit_pos and machine_unit_neg. These values are used
//   in the routine convert_adc2float(...).
// - Initialize Timer3 and Timer5 which are used to control OP time and sampling 
//   intervals.
//***************************************************************************************
void setup() { // Setup interfaces
  int i;
  char buffer[10];            // General purpose buffer, required to read adc to get module_ids

  Serial.begin(BAUDRATE);     // Initialize serial line interfacen
  Serial.setTimeout(TIMEOUT); // The c- and C-commands require an integer as parameter, thus an explicit timeout

  DDRA = B11111111;           // Port A is an output port connected to A0-A7 on the bus
  DDRB = B01000110;           // PB6 = SYNC (Output), PB3 = MISO (Input), PB2 = MOSI (Output), PB1 = SCLK (Output)
  DDRC = B11111111;           // Port C controls the eight digital outputs on the front panel
  DDRE = B11011111;           // Port E controls the various bus control lines and LEDs on the front panel
  DDRG = B00011110;
  DDRH = B11111111;           // Port H is connected to A8-A15 on the bus
  DDRL = B00000000;           // Port L is mapped to the eight digital inputs on the front panel

  D_OUT = 0;                  // Set all digital outputs to 0
  PORTE = PORTE_DEFAULT;
  PORTG = PORTG_DEFAULT;
  SYNC_PORT |= B01000000;     // Deactivate SYNC for the digital potentiometers
  RW_PORT = B00001110;        // N/A, SYNC = 0, N/A, CNV = 0, /WRITE = 1, /READ = 1, /CSADC = 1, N/A

  halt();                     // Switch on the HALT-LED to show that the bus scan is running...

  // Scan the bus to determine the address of the hybrid controller and the addresses of all optional
  // DPT24 modules. These addresses are required to initialize the digital potentiometers of these
  // modules and to build the associated data structures.
  for (i = 0; i < MAX_DPT_MODULES; i++)
    dpt_modules[i].address = dpt_modules[i].number_of_potentiometers = 0;
  i = 0;
  for (unsigned int rack = 0; rack < 0x10; rack++) {
    for (unsigned chassis = 0; chassis < 0x10; chassis++) {
      for (unsigned module = 0; module < 0x10; module++) {
        unsigned int address = (rack << 12) | (chassis << 8) | (module << 4);
        write_address(address);                   // Address the requested computing element
        PORTG &= B11111011;                       // Set the busline /READ = 0
        delayMicroseconds(READ_DELAY);            // This is necessary to allow the buslines to settle - otherwise spurious module ids will be read
        module_id = DBUS_IN;                      // Remember the ID of the module just read out - this is ugly since module_id is global... *sigh*

        if (module_id == MODULE_ID_HC)                      // This is the HC's slot
          dpt_modules[i].number_of_potentiometers = PT_HC;  // A HC has PT_HC digital potentiometers builtin
        else if (module_id == MODULE_ID_DPT24)              // We found a DPT24 module and have to remember its address for initialization
          dpt_modules[i].number_of_potentiometers = PT_DPT24;
        else
          continue;

        dpt_modules[i].address = address;
        dpt_modules[i].module_id = module_id;
#ifdef DEBUG
        Serial.print(String(i) + ": "); Serial.print(dpt_modules[i].address, HEX); Serial.print(": " + String(dpt_modules[i].module_id) + "\n");
#endif

        initialize_dpts(&dpt_modules[i]);  // Initalize all DPTs on this particular board

        i++;
        if (i >= MAX_DPT_MODULES) {
          Serial.print("ERROR: Too many DPT24 modules found!\nMaximum is " + String(MAX_DPT_MODULES) + "\n");
          set_err_led();
          i--;
        }
      }
    }
  }

  ic();                                           // Bus scan complete, switch to initial condition
  clear_err_led();

  init_adc();                                     // Send Soft Span configuration word, this is absolutely necessary for the ADC to work correctly!

  machine_unit_pos = read_adc(M_UNIT_POS_ADDR_OLD, PS_READ_DELAY);
  if (module_id == 0) {                           // module_id is globale, 0 is the id of the PS card
#ifdef DEBUG
    Serial.print("PS card found at old address\n");
#endif
    machine_unit_neg = read_adc(M_UNIT_NEG_ADDR_OLD, PS_READ_DELAY);
  } else {                                        // No PS card at the old address, it must be a new machine
#ifdef DEBUG
    Serial.print("No PS card found at old address - module_id = " + String(module_id) + "\n");
#endif
    machine_unit_pos = read_adc(M_UNIT_POS_ADDR, PS_READ_DELAY);
    if (module_id != 0)
      Serial.print("PANIC: PS card neither found at old nor at new address - module_id = " + String(module_id) + "\n");
    machine_unit_neg = read_adc(M_UNIT_NEG_ADDR, PS_READ_DELAY);
  }
#ifdef DEBUG
  Serial.print("+ = " + String(machine_unit_pos) + ", - = " + String(machine_unit_neg) + "\n");
#endif
  no_of_samples = 0;                              // Clear all data samples gathered previously

  Timer3.stop();
  Timer5.stop();
  Timer5.attachInterrupt(sample_elements);
  Timer3.attachInterrupt(stop_single_run);
}

//***************************************************************************************
//  The following function must be called once for each DPT-module in order to 
// initialize it and its corresponding datastructure properly.
//***************************************************************************************
void initialize_dpts(struct DPT_MODULE *data) {
  int i;

  for (i = 0; i < data->number_of_potentiometers; data->values[i++] = 0x1802);              // Set the builtin digital potentiometers to write-enabled
  data2potentiometers(RAW, data->address, data->values, data->number_of_potentiometers);    // Transmit raw data to potentiometers

  for (i = 0; i < data->number_of_potentiometers; data->values[i++] = 0);                   // Set all wiper positions to zero
  data2potentiometers(COOKED, data->address, data->values, data->number_of_potentiometers); // Transmit wiper position data to potentiometers
}

//***************************************************************************************
//  init_adc() initializes the LTC2357 analog digital converter by sending an apropriate
// Soft Span configuration word with explicit bit banging. This routine must be called
// once before using the ADC. It is normally called from within setup().
//***************************************************************************************
int16_t init_adc() {                        // Initiate a conversion and readout the result from the ADC (LTC2357)
  uint32_t result;                          // This will hold one result value with the lower 24 bits containing the actual bits read
  int16_t value,                            // Actual value read from the 16 bit ADC
          adccfg;                           // ADC configuration bit mask

  RW_PORT |= B00010000;                     // Toggle CNV: First, set CNV = 1, this is independent from /CS
  RW_PORT &= B11101111;                     // Reset CNV - these two successive port manipulations yield a high-time of 120 ns on the Mega2650-CORE
  PORTB   |= B00000010;                     // SCLK = 1
  RW_PORT &= B11111101;                     // Set /CS to low, thus enabling the ADC's serial line interface
  adccfg = 0x70;                            // ADCCONFIG word sets number of channels and SOFTSPAN word
  PORTB &= B11111011;                       // Clear MOSI first

  for (int i = 24; i > 0; i--) {            // Read and write 24 bits all in all

    if (adccfg & 0x8000)
      PORTB |= B00000100;                   // Set MOSI according to the ADCCONFIG word
    else
      PORTB &= B11111011;

    adccfg <<= 1;

    PORTB &= B11111101;                     // Toggle SCLK: SCLK = 0
    PORTB |= B00000010;                     // ...and SCLK = 1
  }

  read_adc(M_UNIT_POS_ADDR, READ_DELAY);    // Perform one dummy read - this is necessary only for some (?!) of the LTC2357 chips...
  read_adc(M_UNIT_POS_ADDR, READ_DELAY);    // Perform one dummy read - this is necessary only for some (?!) of the LTC2357 chips...
}

//***************************************************************************************
//  read_adc(...) reads the output voltage of a computing element at a given address.
// Before using this routine, init_adc() must have been called one (which is done in 
// setup()). 
//  This routine returns a 16 bit integer value which can be converted to a single 
// precision floating point number by calling convert_adc2float(...) subsequently.
//  This routine also sets the global variable module_id to the ID value returned by 
// the element addressed on the data bus. Only the lower 7 bits of the module ID read
// are returned.
//***************************************************************************************
int16_t read_adc(unsigned int address, unsigned int delay) {
  uint8_t rx0, rx1, rx2;

  write_address(address);                             // Address the requested computing element
  assert_read();                                      // Activate the /READ bus line
  delayMicroseconds(delay);                           // Wait a moment for the signal on readout to settle (long bus line)
  module_id = DBUS_IN & 0x7f;                         // Remember the ID of the module just read out - this is ugly since module_id is global... *sigh*
  RW_PORT |= B00010000;                               // Toggle CNV: First, set CNV = 1, this is independent from /CS
  RW_PORT &= B11101111;                               // Reset CNV - these two successive port manipulations yield a high-time of 120 ns on the Mega2650-CORE
  RW_PORT &= B11111101;                               // Set /CS to low, thus enabling the ADC's serial line interface
  delayMicroseconds(2);                               // TODO: This is required only on one of the HCs known today. Does this HC has a too slowly falling /CS edge?

  SPI.beginTransaction(adc_spi);
  rx0 = SPI.transfer(0);                              // Read three single bytes (only the first two contain the actual data,
  rx1 = SPI.transfer(0x70);                           // the third one contains the Soft Span word which is checked
  rx2 = SPI.transfer(0);                              // just to be sure everything worked OK).
  SPI.end();
#ifdef DEBUG
  Serial.print("Result = "); Serial.print(rx0, HEX); Serial.print(" "); Serial.print(rx1, HEX); Serial.print(" "); Serial.print(rx2, HEX); Serial.print("\n");  
#endif
  RW_PORT |= B00000010;                               // Set /CS to high
  deassert_rw();                                      // Deactivate /READ, thus releasing the bus again

  if (rx2 != 0x7) {                                   // The third word must contain the Soft Span configuration word which is 0x7 in our case, just be paranoid
    Serial.print("ERROR: Illegal value read from ADC! ");
    Serial.print("Result = "); Serial.print(rx0, HEX); Serial.print(" "); Serial.print(rx1, HEX); Serial.print(" "); Serial.print(rx2, HEX); Serial.print("\n");  
    Serial.print("\n");
  }

  return (rx0 << 8) | rx1;                            // Combine high and low byte of the result
}

//***************************************************************************************
//  convert_adc2float(...) converts a 16 bit value read from the ADC to a single precision
// floating point number. Therefore the basis values of the positive and negative machine
// units are required which were already read out during setup().
//***************************************************************************************
float convert_adc2float(int16_t value) {
  float result;

  if (value >= 0)
    result = (float) value / (float) machine_unit_pos;
  else
    result = -((float) value / (float) machine_unit_neg);

  return result;
}

//***************************************************************************************
//  write_address(...) writes a 16 bit address to the address bus thus selecting one 
// computing element for a further operation.
//***************************************************************************************
void write_address(unsigned int address) {  // Set the 16 address lines to the address of the computing element to be selected
  PORTA = address & 0xff;                   // Output the eight low bits of the address
  PORTH = (address >> 8) & 0xff;            // Output the upper eight bits
}

//***************************************************************************************
//  data2potentiometers(...) sends data (16 bit per potentiometer) to daisy-chained 
// potentiometers like those contained on the HC and DPT24 modules.
//
//  Parameters:
//    mode:         RAW    = transmit data as is, used for mode setup etc.
//                  COOKED = transmit wiper data
//    address:      Address of the DPT module on the bus. The local DPTs must be 
//                  addressed using the HC's own address
//    data[]:       Array holding the data for all daisy-chained potentiometers of 
//                  board being addressed
//    number_of_pt: Number of entries in the aforementioned array
//***************************************************************************************
void data2potentiometers(int mode, unsigned int address, unsigned int data[], unsigned int number_of_pt) {
  int i;
  unsigned char high, low;

#ifdef DEBUG
  Serial.print("data2potentiometers: number_of_pt = " + String(number_of_pt) + "\n");
#endif

  write_address(address & 0xfff0);      // Address the potentiometer group - the four lowest address bits must be 0!
  assert_write();                       // and make sure /WRITE is active so that MOSI/SCKL etc. are gated, this also ensures that /CS = 1

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);            // This is pretty specific to the digital potentiometers used and should be
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // done before any access to make sure that we have the right settings
  SPI.setDataMode(SPI_MODE1);

  SYNC_PORT &= B10111111;               // Initiate transfer and transfer data to the daisy-chained potentiometers, data for the first one must come last!
  for (i = number_of_pt - 1; i >= 0; i--) {
    high = (data[i] >> 8) & 0xff;
    low  = data[i] & 0xff;

    if (mode != RAW) {                  // Transfer wiper data and update wiper position
      high &= 0x3;
      low  &= 0xff;
      high |= 0x4;
    }

    SPI.transfer(high);
    SPI.transfer(low);
  }

  SYNC_PORT |= B01000000;               // Let the potentiometers know that the data upload is finished
  SPI.end();
  deassert_rw();                        // Deassert /WRITE
}

//***************************************************************************************
//  data2xbar(...) sends a configuration bit stream to the XBAR module at the address
// specified in the first parameter.
//***************************************************************************************
  void data2xbar(unsigned int address, unsigned char *data) {
  int i;

  write_address(address & 0xfff0);      // Address the potentiometer group - the four lowest address bits don't matter
  assert_write();                       // and make sure /WRITE is active so that MOSI/SCKL etc. are gated, this also ensures that /CS = 1

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);            // This is pretty specific to the digital potentiometers used and should be
  SPI.setClockDivider(SPI_CLOCK_DIV2);  // done before any access to make sure that we have the right settings
  SPI.setDataMode(SPI_MODE2);           // The AD8113 runs in SPI-mode 2
  
  SYNC_PORT |= B01000000;               // Load the shift register

  for (i = 0; i < XBAR_CONF_BYTES; i++) {
    SPI.transfer(data[i]);
  }

  SYNC_PORT &= B10111111;               // Activate the new configuration loaded into the shift register
  SPI.end();
  deassert_rw();                        // Deassert /WRITE
}

//***************************************************************************************
//  sample_elements() samples data from all computing elements defined in the current 
// readout-group.
//  It is called by a Timer1-interrupt at regular intervals. The data can then be sent
// to the attached digital computer by the 'l' command.
//***************************************************************************************
void sample_elements() {
  int i;
  uint32_t value;

  if (mode != MODE_OP)  // Reading micros() at the beginning of op() takes too much time
    return;             // Enabling the timer in op() would be to slow, too, so let's make
                        // sure here that we have reached OP_MODE already.

  for (int i = 0; i < ro_group_size; i++) { // Read all elements defined in the readout-group
    samples[no_of_samples++] = read_adc(ro_group[i], READ_DELAY);
#ifdef DEBUG
    Serial.print(String(no_of_samples) + ": " + String(samples[no_of_samples - 1]) + " ");
#endif
    if (no_of_samples == MAX_SAMPLES) {
      Serial.print("Sampling buffer overflow!\n");
      no_of_samples = 0;
    }
  }
#ifdef DEBUG
  Serial.print("\n");
#endif
}

//***************************************************************************************
// ic() switches the analog computer into initial condition mode.
//***************************************************************************************
void ic() {
  op_end = micros();
  PORTE = PORTE_IC;
  mode = MODE_IC;
}

//***************************************************************************************
// op() switches the analog computer into operate mode.
//***************************************************************************************
void op() {
  op_start = micros();  // Remember start time to compute delta-t later
  PORTE = PORTE_OP;
  mode = MODE_OP;
}

//***************************************************************************************
// halt() switches the analog computer into halt mode.
//***************************************************************************************
void halt() {
  op_end = micros();
  PORTE = PORTE_HALT;
  mode = MODE_HALT;
}

//***************************************************************************************
// ps() switches the analog computer into potentiometer set mode.
//***************************************************************************************
void ps() {
  PORTE = PORTE_POTSET;
  mode = MODE_POTSET;
}

//***************************************************************************************
// set_err_led() switches the error LED on.
//***************************************************************************************
void set_err_led() {
  PORTE &= B01111111;
}

//***************************************************************************************
// clear_err_led() switches the error LED off.
//***************************************************************************************
void clear_err_led() {
  PORTE |= B10000000;
}

//***************************************************************************************
// clear_digital_output() clears a specific digital output line (0 to 7).
//***************************************************************************************
void clear_digital_output(unsigned int value) { // Clear a digital output line
  if (value > 7) {
    set_err_led();
    Serial.print("ERR\n");
  } else
    D_OUT &= ~(1 << value) & 0xff;
}

//***************************************************************************************
// set_digital_output() sets a specific digital output line (0 to 7).
//***************************************************************************************
void set_digital_output(unsigned int value) {   // Set a digital output line
  if (value > 7) {
    set_err_led();
    Serial.print("ERR\n");
  } else
    D_OUT |= 1 << value;
}

//***************************************************************************************
// assert_read() activates the /READ control line on the system bus.
//***************************************************************************************
void assert_read() {    // Activate /READ only
  deassert_rw();
  RW_PORT &= B11111011;
}

//***************************************************************************************
// assert_write() activates the /WRITE control line on the system bus.
//***************************************************************************************
void assert_write() {   // Activate /WRITE only
  deassert_rw();
  RW_PORT &= B11110111;
}

//***************************************************************************************
// deassert_rw() deactivates the /READ and /WRITE control lines on the system bus.
//***************************************************************************************
void deassert_rw() {
  RW_PORT |= B00001100; // Set /WRITE and /READ to 1 - the interlock logic then suppresses the signals altogether
}

//***************************************************************************************
//  stop_single_run() is controlled by TimerThree and stops the OP-portion of a 
// single-run when its run time is over.
//***************************************************************************************
void stop_single_run() {
  static int first_call = TRUE;

  if (state != STATE_SINGLE_RUN_OP) // If this is true, the routine has been called spuriously,
    return;                   // which should be ignored.

#ifdef DEBUG
  Serial.print("F=" + String(first_call) + "\nState = " + String(state) + "\n");
#endif
  if (first_call) // The routine is called twice - once at the beginning of a timer period and once at the end, only the last one is relevant
    first_call = FALSE;
  else {
    Timer3.stop();            // Stop the OP-interval timer
    Timer5.stop();            // Stop data gathering
    state = STATE_NORMAL;
    first_call = TRUE;        // Prepare for next call to this routine

    halt();
    if (sync_mode) {
      Serial.print("EOSR\n"); // End of single-run reached
      sync_mode = FALSE;
    }
  }
}

//***************************************************************************************
//  loop() contains the main control logic of the HC module firmware. This is the place
// where commands are read and interpreted and results are sent back to the attached 
// digital computer.
//***************************************************************************************
void loop() {
  static int enable_ovl_halt = FALSE,           // Halt on overload is disabled by default
             enable_ext_halt = FALSE;           // as is halt on external input
  static unsigned long op_time = 0,             // OP-time in microseconds
                       ic_time = 0,             // IC-time in microseconds
                       now,                     // Current time in microeconds, used for single/repetitive run
                       sample_interval;         // Length of sample interval in microseconds
  unsigned int value, i, j,
           total_samples;                   // Number of samples which can be logged during a single-run
  int16_t address;
  int index, potentiometer_address;
  float result,
        ro_group_values[MAX_GROUP_SIZE];
  char input[INPUT_BUFF_LEN], cmd, buffer[10],  // Character buffer for hexadecimal addresses
       *p;                                         // Scratch pointer
  unsigned char xbar_buffer[XBAR_CONF_BYTES];   // Buffer holding the configuration bit stream for an XBAR module

  for (;;) { // This is a tad faster than relying on loop being called repeatedly
    // Take care of halt conditions - this has to be done before any single/repetitive run logic!
    if (mode == MODE_OP) { // Either halt condition only has an effect if the current analog computer mode is OP
      if (!(OVL_PORT & OVL_BIT) && enable_ovl_halt) {
        if (state == STATE_SINGLE_RUN_OP) 
          stop_single_run();
        else
          halt();
        state = STATE_NORMAL;
        Serial.print("\tOverload halt!\n");
      }
      else if ((EXT_HALT_PORT & EXT_HALT_BIT) && enable_ext_halt) {
        halt();
        if (state == STATE_SINGLE_RUN_OP && sync_mode)
          Serial.print("EOSRHLT\n");
        state = STATE_NORMAL;
      }
    }

    // Take care of single and repetitive runs
    if (state == STATE_SINGLE_RUN_IC) {
      if (micros() - now >= ic_time) { // End of IC-period reached
        state = STATE_SINGLE_RUN_OP;

        if (ro_group_size) { // Activate logging if a readout-group has been defined
          no_of_samples = 0;                            // Clear all data samples gathered previously
          total_samples = (MAX_SAMPLES - CONTINGENCY_SAMPLES)  / ro_group_size;
          sample_interval = op_time / total_samples;  // Delta-t for the interrupt routine gathering data
          if (sample_interval < MIN_SAMPLE_INT)       // Make sure we do not sample way too often thus avoiding
            sample_interval = MIN_SAMPLE_INT;         // potential problems with updating micros() etc.
#ifdef DEBUG
          Serial.print("Total samples = " + String(total_samples) + ", Delta-t = " + String(sample_interval) + "\n");
#endif
          Timer5.initialize(sample_interval);
          Timer5.restart();
        }

        now = micros(); // Remember start of OP-period
        op(); // This takes a tick :-) for reading out micros() to determine run time, so let's start the interrupt after calling it
        Timer3.initialize(op_time); // Prepare time to end the OP-interval
        Timer3.restart();
      }
    }
    else if (state == STATE_SINGLE_RUN_OP) {  // This became obsolete due to the interrupt controlled routine stop_single_run()
    }
    else if (state == STATE_REPETITIVE_IC) {
      if (micros() - now >= ic_time) {
        state = STATE_REPETITIVE_OP;
        now = micros();
        op();
      }
    }
    else if (state == STATE_REPETITIVE_OP) {
      if (micros() - now >= op_time) {
        state = STATE_REPETITIVE_IC;
        now = micros();
        ic();
      }
    }

    if (Serial.available() > 0) { // If there is anything on the serial line, read and process it
      clear_err_led();
      switch (cmd = Serial.read()) {
        case 'a': // Disable halt on overload
          enable_ovl_halt = FALSE;
          Serial.print("OVLH=DISABLED\n");
          break;
        case 'A': // Enable halt on overload
          enable_ovl_halt = TRUE;
          Serial.print("OVLH=ENABLED\n");
          break;
        case 'b': // Disable external halt
          enable_ext_halt = FALSE;
          Serial.print("EXTH=DISABLED\n");
          break;
        case 'B': // Enable external halt
          enable_ext_halt = TRUE;
          Serial.print("EXTH=ENABLED\n");
          break;
        case 'c': // Set OP-time, format: c\d{6}, time is in milliseconds
          input[Serial.readBytesUntil('\n', input, TIME_DIGITS)] = 0;
          op_time = strtol(input, 0, 10) * 1000;
          Serial.print("T_OP=" + String(op_time / 1000) + "\n");
          break;
        case 'C': // Set IC-time, format: C\d{6}, time is in milliseconds
          input[Serial.readBytesUntil('\n', input, TIME_DIGITS)] = 0;
          ic_time = strtol(input, 0, 10) * 1000;
          Serial.print("T_IC=" + String(ic_time / 1000) + "\n");
          break;
        case 'd': // Clear digital output, format: d[0-7]
          while (Serial.available() <= 0);  // Blocking IO
          clear_digital_output(Serial.read() - '0');
          break;
        case 'D': // Set digital output, format: D[0-7]
          while (Serial.available() <= 0);  // Blocking IO
          set_digital_output(Serial.read() - '0');
          break;
        case 'e': // Start repetitive operation
          Serial.print("REP-MODE\n");
          state = STATE_REPETITIVE_IC;
          now = micros();
          ic();
          break;
        case 'E': // Start single IC/OP-cycle
          Serial.print("SINGLE-RUN\n");
          state = STATE_SINGLE_RUN_IC;
          now = micros();
          ic();
          break;
        case 'F': // Start single IC/OP-cycle with completion message to synchronize with the Perl library
          Serial.print("SINGLE-RUN\n");
          state = STATE_SINGLE_RUN_IC;
          now = micros();
          ic();
          sync_mode = TRUE;
          break;
        case 'f': // Fetch values for all elements defined in a readout group
          noInterrupts();
          for (int i = 0; i < ro_group_size; i++) // 1st, we read all elements to minimize jitter
            ro_group_values[i] = convert_adc2float(read_adc(ro_group[i], READ_DELAY));

          for (int i = 0; i < ro_group_size; i++) { // 2nd, we output the values
            dtostrf(ro_group_values[i], 4, 4, buffer);
            Serial.print(buffer);
            if (i < ro_group_size - 1)
              Serial.print(";");
          }
          Serial.print("\n");
          interrupts();
          break;
        case 'G': // Define a readout group consisting of hex addresses delimited by semicola
          ro_group_size = 0;
          input[Serial.readBytesUntil('.', input, MAX_GROUP_SIZE * 5)] = 0;
          p = strtok(input, ";");
          while (p) {
            ro_group[ro_group_size++] = strtol(p, 0, 16);
            p = strtok(NULL, ";");
          }

          no_of_samples = 0;  // After defining a new readout-group there is no way to use old sampled data
          break;
        case 'g': // Set address of a computing element and return its ID and associated value, format: g\x{4}
          input[Serial.readBytesUntil('\n', input, 4)] = 0;
          address = strtol(input, 0, 16);
          result = convert_adc2float(read_adc(address, READ_DELAY));
          dtostrf(result, 4, 4, buffer);
          Serial.print(buffer);
          Serial.print(" " + String(module_id) + "\n");
          break;
        case 'h': // Set analog computer to halt
          Serial.print("HALT\n");
          state = STATE_NORMAL;  // Make sure to kill any active single/repetitive run
          halt();
          break;
        case 'i': // Set analog computer to initial condition
          Serial.print("IC\n");
          state = STATE_NORMAL;  // Make sure to kill any active single/repetitive run
          ic();
          break;
        case 'L': // Locate a computing element
          input[Serial.readBytesUntil('\n', input, 4)] = 0;             // Now read the element's address
          address = strtol(input, 0, 16);
          write_address(address);                                       // Address the requested computing element
          if (address == 0xffff)
            PORTG |= B00000100;                                         // Deassert /READ
          else
            PORTG &= B11111011;                                         // Set the busline /READ = 0, thus turning the read LED on
          break;
        case 'l': // Dump samples from last single-run
          if (!ro_group_size || !no_of_samples)
            Serial.print("No data!\n");
          else {
            for (i = 0; i < no_of_samples; i++) {
              dtostrf(convert_adc2float(samples[i]), 4, 4, buffer);
              Serial.print(buffer); Serial.print(" ");
              if (ro_group_size == 1 || (i && !((i + 1) % ro_group_size)))
                Serial.print("\n");
            }
            Serial.print("EOD\n");
          }
          break;
        case 'o': // Set analog computer to operate mode
          Serial.print("OP\n");
          state = STATE_NORMAL;  // Make sure to kill any active single/repetitive run
          op();
          break;
        case 'P': // Set a digital potentiometer to a value, format: P\x{4}\x{2}\d{4}
          input[Serial.readBytesUntil('\n', input, 4)] = 0;             // Read four hex digits for the potentiometer card's address
          address = strtol(input, 0, 16);
          input[Serial.readBytesUntil('\n', input, 2)] = 0;             // Now read the number of the potentiometer on the card to be set. Since some
          potentiometer_address = strtol(input, 0, 16);                 // cards can have > 16 potentiometers, this cannot be part of the card's address!
          input[Serial.readBytesUntil('\n', input, PT_VAL_DIGITS)] = 0; // Read the value the potentiometer should be set to - this is a decimal
          value = strtol(input, 0, 10);                                 // value - maybe I should rewrite everything and use hexadecimal consistently.

          index = -1;
          for (i = 0; i < MAX_DPT_MODULES; i++) {                       // Determine the index to the dpt_modules structure array
            if (dpt_modules[i].address == address) {
              index = i;
              break;
            }
          }

          if (index == -1) {  // No module containing digital potentiometers at this address found!
            Serial.print("P"); Serial.print(address, HEX); Serial.print("."); Serial.print(potentiometer_address, HEX); Serial.print("=ERROR!\n");
          } else {
            dpt_modules[index].values[potentiometer_address] = value & PT_VAL_MASK;
            data2potentiometers(COOKED, dpt_modules[i].address, dpt_modules[index].values, dpt_modules[index].number_of_potentiometers); // Send wiper position data, not raw data!
            Serial.print("P"); Serial.print(address, HEX); Serial.print("."); Serial.print(potentiometer_address, HEX); Serial.print("=");
            Serial.print(String(dpt_modules[i].values[potentiometer_address]) + "\n");
          }
          break;
        case 'q': // Dump digital potentiometer settings
          for (i = 0; i < MAX_DPT_MODULES; i++) {
            if (!dpt_modules[i].address) break; // The first address of 0 terminates the list of DPT-modules
            if (i > 0) Serial.print(";");
            Serial.print(dpt_modules[i].address, HEX);
            Serial.print(":");
            for (int j = 0; j < dpt_modules[i].number_of_potentiometers; j++) {
              if (j > 0) Serial.print(",");
              Serial.print(dpt_modules[i].values[j]);
            }
          }
          Serial.print("\n");
          break;
        case 'R': // Read digital inputs and return their associated values
          value = D_IN;
          for (i = 0; i < 8; i++) Serial.print(String((value >> i) & 1) + " ");
          Serial.print("\n");
          break;
        case 's': // Print system status
          Serial.print("STATE=");
          switch (state) {
            case STATE_NORMAL:
              Serial.print("NORM");
              break;
            case STATE_SINGLE_RUN_IC:
              Serial.print("SR-IC");
              break;
            case STATE_SINGLE_RUN_OP:
              Serial.print("SR-OP");
              break;
            case STATE_REPETITIVE_IC:
              Serial.print("REP-IC");
              break;
            case STATE_REPETITIVE_OP:
              Serial.print("REP-OP");
          }
          Serial.print(",+1=" + String(convert_adc2float(machine_unit_pos)) + ",-1=" + String(convert_adc2float(machine_unit_neg)));
          Serial.print(",\
MODE=" + (mode == MODE_IC ? String("IC") : (mode == MODE_OP ? String("OP") : String("HALT"))) + ",\
EXTH=" + (enable_ext_halt ? String("ENA") : String("DIS")) + ",\
OVLH=" + (enable_ovl_halt ? String("ENA") : String("DIS")) + ",\
IC-time=" + String(ic_time / 1000) + ",\
OP-time=" + String(op_time / 1000) + ",RO-GROUP=");
          for (int i = 0; i < ro_group_size; i++) {
            Serial.print(ro_group[i], HEX);
            if (i < ro_group_size - 1)
              Serial.print(";");
          }
          Serial.print(",DPTADDR=");
          for (int i = 0; i < MAX_DPT_MODULES; i++) {
            if (!dpt_modules[i].address) break; // The first address of 0 terminates the list of DPT-modules
            if (i > 0) Serial.print(";"); // Column separator if this is not the first column
            Serial.print(dpt_modules[i].address, HEX);
            Serial.print("/" + String(dpt_modules[i].module_id));
          }
          Serial.print("\n");
          break;
        case 'S': // Switch the analog computer to POTSET-mode
          Serial.print("PS\n");
          state = STATE_NORMAL;  // Make sure to kill any active single/repetitive run
          ps();
          break;
        case 't': // Print current time spent in OP-mode
          Serial.print("t_OP=");
          if (op_start == -1)       // We were never in OP-mode, so there is nothing to display
            Serial.print("N/A");
          else if (mode == MODE_OP)  // The analog computer is currently in OP-mode, so time is still running
            Serial.print(String((micros() - op_start) / 1000));
          else                      // We are no longer in OP-mode, use the stored end-time instead of current time
            Serial.print(String((op_end - op_start) / 1000));
          Serial.print("\n");
          break;
        case 'x': // Reset hybrid controller
          setup();
          ro_group_size = 0;
          Serial.print("RESET\n");
          break;
        case 'X': // Configure an XBAR-module
          input[Serial.readBytesUntil('\n', input, 4)] = 0; // Read four hex digits for the XBAR-module's address
          address = strtol(input, 0, 16);

          for (i = 0; i < XBAR_CONF_BYTES; i++) {           // Read configuration data in pairs of hex nibbles and store it
            input[Serial.readBytesUntil('\n', input, 2)] = 0;
            xbar_buffer[i] = strtol(input, 0, 16);
          }

          data2xbar(address, xbar_buffer);
          Serial.print("XBAR READY\n");
          break;
        case '?': // Print help
          Serial.print("HC firmware " + String(VERSION) + " " + String(VERSION_DATE));
          Serial.print("\n\nCommands:\n\
  a           Disable halt-on-overflow\n\
  A           Enable halt-on-overflow\n\
  b           Disable external halt\n\
  B           Enable external halt\n\
  c\\d{6}      Set OP time for repetitive/single operation\n\
  C\\d{6}      Set IC time for repetitive/single operation\n\
  d[0-7]      Clear digital output\n\
  D[0-7]      Set digital output\n\
  e           Start repetitive operation\n\
  E           Start single IC/OP-cycle\n\
  f           Get (fetch) data for a readout group defined by an address list via G\n\
  F           Start single IC/OP-cycle with completion message (for sync operation)\n\
  g\\x{4}      Set address of computing element and return its ID and value\n\
  G\\w         Set addresses \\x;\\x. for group readout, '.' terminates the input\n\
  h           Halt\n\
  i           Initial condition\n\
  l           Get data from last logging operation\n\
  L\\x{4}      Locate a computing element by its 4 digit address by turning on the read LED\n\
              An address of ffff will deassert read.\n\
  o           Operate\n\
  P\\x{4}\\x{2}\\d{4} Set the digital potentiometer number \\x{2} on the card with\n\
              address \\x{4} to value \\d{4}\n\
  q           Dump digital potentiometer settings\n\
  R           Read digital inputs\n\
  s           Print status\n\
  S           Switch to PotSet-mode\n\
  t           Print elapsed OP-time\n\
  x           Reset\n\
  X\\x{4}\\x{20} Send a configuration bitstream (40 hex nibbles) to the XBAR-module at address \\x{4}\n\
  ?           Print Help\n\n");
          break;
        default:  // Illegal command
          set_err_led();
          Serial.print("Illegal command: ");
          Serial.print(cmd, HEX);
          Serial.print("\n");
      }
    }
  }
}

