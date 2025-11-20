# Remote Switch Feature - Xbox Controller

## Overview
This implementation adds a remote control switch to enable/disable the policy actions using an Xbox Series X controller.

## Features
- **Default State**: Policy is DISABLED - all actions are set to zero (0.0f)
- **Button A**: Enable policy - uses real actions from the neural network
- **Button B**: Disable policy - sets all actions to zero
- **Button X**: Emergency motor disable - calls `disableMotor()`
- **Button Y**: Emergency motor disable - calls `disableMotor()`

## Implementation Details

### Variables Added
```cpp
bool policy_enabled = false;  // Default: actions are all zeros
bool btnA_prev = false;       // Track previous button state (for edge detection)
bool btnB_prev = false;       // Track previous button state (for edge detection)
bool btnX_prev = false;       // Track previous button state (for edge detection)
bool btnY_prev = false;       // Track previous button state (for edge detection)
```

### Button Control Logic
The Xbox controller is polled every loop cycle (20Hz):
1. `xboxController.onLoop()` is called to update controller state
2. Button states are checked for rising edge (button press, not hold)
3. **A button press**: Sets `policy_enabled = true`
4. **B button press**: Sets `policy_enabled = false`
5. **X button press**: Calls `disableMotor()` - hardware motor shutdown
6. **Y button press**: Calls `disableMotor()` - hardware motor shutdown
7. If `policy_enabled == false`, all actions are overridden to 0.0f

### Status Display
The serial monitor shows the current policy state:
- `✓ ENABLED (Actions from policy)` - Policy is active
- `✗ DISABLED (Actions = 0) - Press A to enable` - Policy is disabled

## Usage

1. **Connect Xbox Controller**: Wait for controller to pair via Bluetooth
2. **Start with Safety**: Default state is DISABLED (motors won't move)
3. **Enable Motors**: Press **A button** to start using policy actions
4. **Emergency Stop (Soft)**: Press **B button** to disable policy (sets actions to zero)
5. **Emergency Stop (Hard)**: Press **X or Y button** to disable motor hardware
6. **Re-enable**: Press **A button** again to resume policy

## Safety Features

- **Safe Default**: System starts with policy disabled (actions = 0)
- **Immediate Response**: Button presses are detected on rising edge
- **Visual Feedback**: Serial monitor shows current state every loop
- **Two-Level Emergency Stop**:
  - **Soft Stop (B)**: Disables policy, sets actions to zero
  - **Hard Stop (X/Y)**: Calls `disableMotor()` for hardware-level shutdown

## Technical Notes

- Button detection uses edge triggering (not level triggering) to prevent repeated toggles
- Xbox controller state is checked before applying actions
- Works alongside existing recover mode and ESP-NOW communication
- Compatible with all three motor outputs (local, board1, board2)

## Button Mapping Summary

| Button | Function | Description |
|--------|----------|-------------|
| **A** | Enable Policy | Activates neural network actions |
| **B** | Disable Policy | Sets all actions to zero (soft stop) |
| **X** | Motor Disable | Hardware motor shutdown (emergency) |
| **Y** | Motor Disable | Hardware motor shutdown (emergency) |

## Testing

1. Monitor serial output to verify controller connection
2. Test **A button** - should see "✓ POLICY ENABLED" message
3. Test **B button** - should see "✗ POLICY DISABLED" message
4. Test **X button** - should see "⚠ MOTOR DISABLED - X button pressed"
5. Test **Y button** - should see "⚠ MOTOR DISABLED - Y button pressed"
6. Verify motors respond when enabled, stop when disabled
7. Check that policy status is displayed correctly in serial output

## Troubleshooting

- **Controller not responding**: Check Bluetooth pairing
- **Actions still zero with policy enabled**: Verify controller is fully connected (not waiting for first notification)
- **Buttons not working**: Check serial monitor for controller connection status
