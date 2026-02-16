# aip_urdf_compiler

## Overview

The aip_urdf_compiler is a standalone Python script for generating URDF (Unified Robot Description Format) files from configuration files. It simplifies sensor model management by automatically generating URDF models from sensor configurations.

**Note:** This is no longer a ROS 2 package. Users must manually invoke the Python script to generate URDF files.

## Key Features

- URDF generation from YAML configuration files
- Automated sensor transform processing
- Support for multiple sensor types and configurations
- Template-based URDF generation

## Usage

### Manual Script Invocation

To generate URDF files for your description package, manually run the `compile_urdf.py` script:

```bash
python3 scripts/compile_urdf.py <calibration_directory> <output_directory>
```

**Parameters:**

- `<calibration_directory>`: Path to your config directory containing calibration YAML files
- `<output_directory>`: Path to your URDF output directory

**Example:**

```bash
python3 aip_urdf_compiler/scripts/compile_urdf.py \
    aip_urdf_compiler/example_config \
    aip_urdf_compiler/example_urdf
```

**Note:** The template directory is automatically resolved relative to the script location (`../templates` from the script).

### Configuration Files

The calibration directory must contain the following YAML files:

1. `sensors_calibration.yaml` - Main calibration file defining vehicle-level sensors and sensor units
2. `{unit_name}_calibration.yaml` - Separate calibration files for each sensor unit

**About `unit_name`:**

- When a sensor in `sensors_calibration.yaml` has `type: units`, it's treated as a sensor unit (not an individual sensor)
- The `unit_name` is derived from the child frame name by removing `_base_link` or `_link` suffixes
- Example: If child frame is `sensor_kit_base_link` with `type: units`, the script looks for `sensor_kit_calibration.yaml`
- The sensor unit's calibration file defines all sensors mounted within that unit

**Example structure:**

```yaml
# sensors_calibration.yaml
base_link:
  sensor_kit_base_link: # child frame name
    x: 0.9
    y: 0.0
    z: 2.0
    roll: 0.0
    pitch: 0.0
    yaw: 0.0
    type: units # Marks this as a sensor unit
```

This would require a corresponding `sensor_kit_calibration.yaml` file defining the sensors within that unit.

Each sensor configuration requires:

- `x`, `y`, `z`: Translation values
- `roll`, `pitch`, `yaw`: Rotation values (in radians)
- `type`: Required string, corresponding to a value from [existing sensors](#existing-sensors)
- `frame_id`: Optional string, overwrites the TF frame ID (if not provided, defaults based on sensor type)

### Existing Sensors

```python
class LinkType(enum.Enum):
    """Enum class for the type of the link."""

    CAMERA = "monocular_camera"
    IMU = "imu"
    LIVOX = "livox_horizon"
    PANDAR_40P = "pandar_40p"
    PANDAR_OT128 = "pandar_ot128"
    PANDAR_XT32 = "pandar_xt32"
    PANDAR_QT = "pandar_qt"
    PANDAR_QT128 = "pandar_qt128"
    VELODYNE16 = "velodyne_16"
    VLS128 = "velodyne_128"
    RADAR = "radar"
    GNSS = "gnss"
    JOINT_UNITS = "units"
```

## Architecture

### Components

1. **compile_urdf.py**
   - Main Python script for URDF generation
   - Configuration parser
   - Transform processor
   - URDF generator from templates

2. **Templates**
   - `sensors.xacro.template`: Template for vehicle-level URDF generation (main sensors file)
   - `sensor_unit.xacro.template`: Template for individual sensor unit URDF generation (e.g., sensor kits)

### Generation Process

1. **Configuration Reading**
   - Parses configuration YAML files from the calibration directory
   - Extracts transformation data
   - Validates configurations

2. **Transform Processing**
   - Processes each sensor transform from `sensors_calibration.yaml`
   - Determines sensor types and frame IDs (auto-detects from name if type not specified)
   - Identifies sensor units (type: "units") vs individual sensors
   - Generates appropriate xacro macro strings
   - Creates `sensors.xacro` in the output directory

3. **Sensor Unit Processing**
   - For each sensor unit found in main calibration file
   - Loads corresponding `{unit_name}_calibration.yaml` file
   - Processes all sensors within that unit
   - Generates separate `{unit_name}.xacro` file in the output directory (e.g., `sensor_kit.xacro`)

## Adding New Sensors

1. Add sensor descriptions (xacro module files) in either:
   - Your target package
   - `common_sensor_description` package or other sensor description packages

2. Update the following in `compile_urdf.py`:
   - `LinkType` enumeration: Add new sensor type
   - `link_dict` mapping: Map the new sensor type to its corresponding xacro macro

## Troubleshooting

### Debug Output

The script prints status messages during execution. Check the console output for debugging information about:

- Files being processed
- Sensors being generated
- Any errors or warnings

### Common Issues

1. Missing sensor definitions
   - Ensure sensor type is defined in `LinkType` enum in `compile_urdf.py`
   - Verify xacro file exists in the appropriate description package

2. Configuration errors
   - Check that YAML files are properly formatted
   - Verify all required fields (`type`, transform values) are present in configurations

3. Template not found
   - Ensure template directory path is correct
   - Verify template files exist in the templates directory

4. TF Trees errors
   - Check frame_id values in configuration YAML files
   - Verify transform chain completeness

## Contributing

1. Follow Python coding standards
2. Test URDF generation with various configurations
3. Update documentation for new features
4. Ensure templates are compatible with new sensor types
