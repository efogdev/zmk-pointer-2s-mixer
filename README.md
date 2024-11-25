# Input Processor Mixer

This module interrupts and mixes incoming input events from Zephye input subsystem for ZMK.

## What it does

To Be Determined

## Installation

Include this modulr on your ZMK's west manifest in `config/west.yml`:

```yaml
manifest:
  remotes:
    #...
    # START #####
    - name: badjeff
      url-base: https://github.com/badjeff
    # END #######
    #...
  projects:
    #...
    # START #####
    - name: zmk-input-processor-mixer
      remote: badjeff
      revision: main
    # END #######
    #...
```

Roughly, `overlay` of a 2-sensors trackball should look like below.

```
#include <dt-bindings/zmk/input_mixer.h>
#include <input/processors/mixer.dtsi>

//** To unify a mixture from various input devices,
//   wait and slap all axises as one event sync per 1ms interval.
//   might need to bump up to 12-16ms on wireless peripheral.
// &zip_mixer { sync_report_ms = <1>; };

/{

  //** routing trackball x-asix sensor `pd0` to `&zip_mixer`
  tball1_pri_mmv_il {
    compatible = "zmk,input-listener";
    device = <&pd0>;
    default {
      layers = <DEFAULT>;
      input-processors 
        = <&zip_xy_transform (INPUT_TRANSFORM_X_INVERT)>
        , <&zip_mixer (INPUT_MIXER_X_ONLY)>
        ;
    };
  };

  //** routing trackball y-asix sensor `pd0` to `&zip_mixer`
  tball1_sec_mmv_il {
    compatible = "zmk,input-listener";
    device = <&pd0a>;
    default {
      layers = <DEFAULT>;
      input-processors 
        = <&zip_xy_swap_mapper>
        , <&zip_xy_transform (INPUT_TRANSFORM_Y_INVERT)>
        , <&zip_mixer (INPUT_MIXER_Y_ONLY)>
        ;
    };
  };

  //** routing mixer device as a valid output
  mixin_mmv_il {
    compatible = "zmk,input-listener";
    device = <&zip_mixer>;
  };

};
```
