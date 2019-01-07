# RB3201-RBControl-library

Library for board [RB3201-RBControl](https://github.com/RoboticsBrno/RB3201-RBControl) based on ESP32.

[Library documentation](https://technika.tasemnice.eu/docs/rbcontrol).

Arduino compatible. Available on the [PlatformIO](https://platformio.org/lib/show/5532/RB3201-RBControl).

## Example

```cpp
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <stdio.h>

#include "RBControl_manager.hpp"
#include "RBControl_battery.hpp"

extern "C" void app_main() {
    rb::Manager man;    // Initialize the robot manager

    // Set motor power limits
    man.setMotors()
        .pwmMaxPercent(0, 70)  // left wheel
        .pwmMaxPercent(1, 70)  // right wheel
        .pwmMaxPercent(2, 28)  // turret left/right
        .pwmMaxPercent(3, 45)  // turret up/down
        .set();

    man.leds().yellow(); // Turn the yellow led on

    //man.piezo().setTune(444); // start the piezo

    int i = 0;
    const auto& bat = man.battery();
    while(true) {
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        print("Tick #%d, battery at %d%%, %dmv\n", i++, bat.pct(), bat.voltageMv());
    }
}

```

## Testing the library during development

When you want to test the library, when you making some change in the library, use command `pio ci` in the command line/shell:

```sh
pio ci --lib="." -C "platformio.ini" examples/motors
```

For testing you can use any project in the folder `examples`. Just replace the `examples/motors` with some other project.