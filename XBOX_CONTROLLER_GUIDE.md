# Xbox Controller Quick Reference

## Button Layout
```
        [Y]
    [X]     [B]
        [A]
```

## Button Functions

### Primary Controls
- **[A] Enable Policy**
  - Starts neural network control
  - Motors follow policy actions
  - Serial: "✓ POLICY ENABLED - Using real actions from policy"

- **[B] Disable Policy**
  - Stops policy control (soft stop)
  - Sets all actions to 0.0f
  - Motors receive zero commands
  - Serial: "✗ POLICY DISABLED - Actions set to zero"

### Emergency Controls
- **[X] Motor Disable**
  - Hardware-level motor shutdown
  - Calls `disableMotor()` function
  - More aggressive than B button
  - Serial: "⚠ MOTOR DISABLED - X button pressed"

- **[Y] Motor Disable**
  - Same as X button
  - Hardware-level motor shutdown
  - Calls `disableMotor()` function
  - Serial: "⚠ MOTOR DISABLED - Y button pressed"

## Usage Flow

```
1. System Starts
   └─> Policy DISABLED (safe default)

2. Press [A]
   └─> Policy ENABLED
       └─> Motors follow neural network

3a. Press [B] (Soft Stop)
    └─> Policy DISABLED
        └─> Actions set to zero
        
3b. Press [X] or [Y] (Hard Stop)
    └─> Motor hardware disabled
        └─> Complete shutdown

4. Press [A] again
   └─> Resume operation
```

## Safety Levels

| Level | Button | Action | Use Case |
|-------|--------|--------|----------|
| Normal Stop | B | Policy OFF, actions=0 | Normal stop, can resume quickly |
| Emergency Stop | X or Y | Motor hardware OFF | Emergency situations, hardware shutdown |

## Notes

- All buttons use **edge detection** (press, not hold)
- Controller must be connected and initialized
- Visual feedback in serial monitor for all actions
- Default state is DISABLED for safety
