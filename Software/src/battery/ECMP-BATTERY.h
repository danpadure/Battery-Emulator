#ifndef STELLANTIS_ECMP_BATTERY_H
#define STELLANTIS_ECMP_BATTERY_H
#include "CanBattery.h"
#include "ECMP-HTML.h"

//#define SIMULATE_ENTIRE_VEHICLE_ECMP
//Enable this to simulate the whole car (useful for when using external diagnostic tools)

class EcmpBattery : public CanBattery {
 public:
  // Default constructor – used for primary battery
  EcmpBattery()
      : renderer(&datalayer_extended.stellantisECMP) {
    datalayer_battery              = &datalayer.battery;
    datalayer_battery_extended     = &datalayer_extended.stellantisECMP;
    allows_contactor_closing       = &datalayer.system.status.battery_allows_contactor_closing;
    contactor_closing_allowed      = nullptr;
  }

  // Parameterized constructor – used for secondary battery
  EcmpBattery(DATALAYER_BATTERY_TYPE* datalayer_ptr,
              DATALAYER_INFO_ECMP* extended_ptr,
              bool* contactor_closing_allowed_ptr,
              CAN_Interface targetCan = CAN_BATTERY)
      : CanBattery(targetCan),
        renderer(extended_ptr) {
    datalayer_battery              = datalayer_ptr;
    datalayer_battery_extended     = extended_ptr;
    allows_contactor_closing       = nullptr;  // secondary doesn't control main flag
    contactor_closing_allowed      = contactor_closing_allowed_ptr;
    battery_voltage                = 0;
  }

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "Stellantis ECMP battery";

  bool supports_clear_isolation() { return true; }
  void clear_isolation() { datalayer_battery_extended->UserRequestIsolationReset = true; }

  bool supports_factory_mode_method() { return true; }
  void set_factory_mode() { datalayer_battery_extended->UserRequestDisableIsoMonitoring = true; }

  bool supports_reset_crash() { return true; }
  void reset_crash() { datalayer_battery_extended->UserRequestCollisionReset = true; }

  bool supports_contactor_reset() { return true; }
  void reset_contactor() { datalayer_battery_extended->UserRequestContactorReset = true; }

  bool supports_reset_DTC() { return true; }
  void reset_DTC() { datalayer_battery_extended->UserRequestDTCreset = true; }

  BatteryHtmlRenderer& get_status_renderer() { return renderer; }

 private:
  EcmpHtmlRenderer renderer;

  // Pointers for double-battery support
  DATALAYER_BATTERY_TYPE* datalayer_battery = nullptr;
  DATALAYER_INFO_ECMP*    datalayer_battery_extended = nullptr;
  bool* allows_contactor_closing     = nullptr;
  bool* contactor_closing_allowed    = nullptr;

  static const int MAX_PACK_VOLTAGE_DV = 4546;
  static const int MIN_PACK_VOLTAGE_DV = 3580;
  static const int MAX_CELL_DEVIATION_MV = 100;
  static const int MAX_CELL_VOLTAGE_MV = 4250;
  static const int MIN_CELL_VOLTAGE_MV = 3280;

  // All your original members (CAN frames, timers, PID vars, etc.) go here unchanged
  unsigned long previousMillis10 = 0;
  unsigned long previousMillis20 = 0;
  unsigned long previousMillis50 = 0;
  unsigned long previousMillis100 = 0;
  unsigned long previousMillis250 = 0;
  unsigned long previousMillis500 = 0;
  unsigned long previousMillis1000 = 0;
  unsigned long previousMillis5000 = 0;

  CAN_frame ECMP_010 = {.FD = false, .ext_ID = false, .DLC = 1, .ID = 0x010, .data = {0xB4}};
  // ... (insert all your CAN_frame definitions here, unchanged) ...

  static const uint16_t PID_PACK_VOLTAGE = 0xD815;
  static const uint16_t PID_HIGH_CELL_VOLTAGE = 0xD870;
  static const uint16_t PID_LOW_CELL_VOLTAGE = 0xD86F;
  static const uint16_t PID_AVG_CELL_VOLTAGE = 0xD43D;
  static const uint16_t PID_CURRENT = 0xD816;
  // ... (insert the full block of static const uint16_t PID_... = 0x.... here from your original file) ...

  uint16_t battery_voltage = 370;
  uint16_t battery_soc = 0;
  // ... (all your other member variables, arrays, bools, etc. unchanged) ...
};

#endif
