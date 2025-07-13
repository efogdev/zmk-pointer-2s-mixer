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
#include <dt-bindings/zmk/p2sm.h>
#include <input/processors/p2sm.dtsi>
```

### 2. Define the mixer device

```c
&zip_2s_mixer {
    status = "okay";
	sync-report-ms = <4>; 
	sync-report-yaw-ms = <8>;

	yaw-interference-thres = <24>; // CPI and sync-report-yaw-ms dependent
	yaw-thres = <36>; // CPI and sync-report-yaw-ms dependent

	// arbitrary bounding box (0, 0, 0) - (255, 255, 255)
	ball-radius = <102>; // maximum = 127
	sensor1-pos = [31 4B 2D];  // X Y Z
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

## Kconfig Options

Enable the module in your configuration:

```
CONFIG_ZMK_POINTER_2S_MIXER=y
```

The module is automatically enabled when `CONFIG_ZMK_POINTING=y` is set.
