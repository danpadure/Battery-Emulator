#ifndef _ECMP_BATTERY_HTML_H
#define _ECMP_BATTERY_HTML_H

#include <cstring>
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/webserver/BatteryHtmlRenderer.h"

class EcmpHtmlRenderer : public BatteryHtmlRenderer {
 public:
  // Added: constructor that takes the extended pointer
  EcmpHtmlRenderer(DATALAYER_INFO_ECMP* dl) : ecmp_datalayer(dl) {}

  String get_status_html() {
    String content;

    // Lambda that uses the passed reference (no more direct datalayer_extended access)
    auto print_ecmp = [&content](DATALAYER_INFO_ECMP& data) {
      content += "<h4>Main Connector State: ";
      if (data.MainConnectorState == 0) {
        content += "Contactors open</h4>";
      } else if (data.MainConnectorState == 0x01) {
        content += "Precharged</h4>";
      } else {
        content += "Invalid</h4>";
      }
      // ... (all your other content += lines, but replace EVERY datalayer_extended.stellantisECMP.XXX with data.XXX)
      // Example:
      content += "<h4>Insulation Resistance: " + String(data.InsulationResistance) + "kOhm</h4>";
      // ... continue for EVERY field ...
      // For MysteryVan block:
      if (data.MysteryVan) {
        content += "<h3>MysteryVan platform detected!</h3>";
        // ... use data.CONTACTORS_STATE, data.CrashMemorized, etc. ...
      }
      // ... rest of your HTML ...
    };

    // Call the lambda with the pointed-to data
    print_ecmp(*ecmp_datalayer);

    return content;
  }

 private:
  DATALAYER_INFO_ECMP* ecmp_datalayer;  // This member is required
};

#endif
