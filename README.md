# ZMK Pointer 2-Sensor Mixer

A ZMK module that combines input from two mouse sensors to create cursor and scroll movements. This module calculates
the 3D geometry of sensor positions on a ball and maps their individual movements to a common reference frame.

Inspired by https://github.com/badjeff/zmk-input-processor-mixer

## Installation

Add this module to your ZMK config by including it in your `west.yml`:

```yaml
manifest:
  remotes:
      ...
      - name: efogdev
        url-base: https://github.com/efogdev

  projects:
      ...
      - name: zmk-pointer-2s-mixer
        remote: efogdev
        revision: main
```

## Device Tree Configuration

### 1. Include required headers

```c
#include <dt-bindings/zmk/input_mixer.h>
#include <input/processors/mixer.dtsi>
```

### 2. Define the mixer device

```c
&zip_2s_mixer {
    status = "okay";
    sync-report-ms = <2>;
    sync-report-yaw-ms = <8>;
    ball-radius = <102>; // 0 to 127
    
    yaw-div = <4>; // divider
    yaw-mul = <1>; // multiplier

    // CPI dependent values
    yaw-interference-thres = <6>; // increase this value to descrease interference 
    yaw-thres = <2>; // filters out minimal twist movements 
    
    // zero = left down bottom
    sensor1-pos = [31 4B 2D]; // X, Y, Z 
    sensor2-pos = [C1 3C 2D];
};
```

### 3. Configure input listeners

```c
trackball_primary_listener: trackball_primary_listener {
    compatible = "zmk,input-listener";
    device = <&trackball_primary>;

    listener {
        layers = <DEFAULT>;
        input-processors = <&zip_2s_mixer INPUT_MIXER_SENSOR1>;
        process-next;
    };
};

trackball_secondary_listener: trackball_secondary_listener {
    compatible = "zmk,input-listener";
    device = <&trackball_secondary>;

    listener {
        layers = <DEFAULT>;
        input-processors = <&zip_2s_mixer INPUT_MIXER_SENSOR2>;
        process-next;
    };
};
```

### 4. Define output node

```c
output_node {
    compatible = "zmk,input-listener";
    device = <&zip_2s_mixer>;
};
```

## Example Usage

See the [complete example](https://github.com/efogdev/trackball-zmk-config) in `efogtech_trackball_0.dts` board.

![image](https://github.com/user-attachments/assets/86a2420e-6595-49a2-b843-836f6a7a4053)

## Kconfig Options

Enable the module in your configuration:

```
CONFIG_ZMK_POINTER_2S_MIXER=y
```

The module is automatically enabled when `CONFIG_ZMK_POINTING=y` is set.
