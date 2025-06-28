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
    ball-radius = <107>; // 0 to 127
    
    yaw-div = <32>;
    yaw-interference-thres = <42>;   
    
    // zero = left down bottom 
    sensor1-pos = [00 6F 00]; // X, Y, Z 
    sensor2-pos = [ff 56 00];
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

## Configuration Properties

| Property                     | Type     | Description                                                                                  | Default    |
|------------------------------|----------|----------------------------------------------------------------------------------------------|------------|
| `sync-report-ms`             | uint32   | Synchronization report interval in milliseconds                                              | 4          |
| `sync-report-yaw-ms`         | uint32   | Yaw report interval in milliseconds                                                          | 8          |
| `yaw-div`                    | uint32   | Yaw divisor                                                                                  | 48         |
| `yaw-mul`                    | uint32   | Yaw multiplier                                                                               | 1          |
| `yaw-interference-threshold` | uint32   | Value from 0 to 128, increate to allow more interference between scroll and cursor movements | 42         |
| `ball-radius`                | uint32   | Radius of the trackball in arbitrary units                                                   | 100        |
| `sensor1-pos`                | uint8[3] | Position of sensor [X, Y, Y] relative left down bottom corner of your arbitrary bounding box | [00 80 00] |
| `sensor2-pos`                | uint8[3] |                                                                                              | [ff 80 00] |

## Example Usage

See the complete example in `efogtech_trackball_0.dts` board, it features:

- Two PMW3610 trackball sensors on separate SPI buses
- Proper mixer configuration with ball radius of 107 units
- Sensor positions at [00 6F 00] and [ff 56 00]

![Screenshot From 2025-06-28 18-02-20](https://github.com/user-attachments/assets/5413240c-485c-46e5-8a8b-307fdae793ed)

![Screenshot From 2025-06-28 18-02-48](https://github.com/user-attachments/assets/237f5441-d16a-464c-acfd-f3458e24ed01)

## Kconfig Options

Enable the module in your configuration:

```
CONFIG_ZMK_POINTER_2S_MIXER=y
```

The module is automatically enabled when `CONFIG_ZMK_POINTING=y` is set.
