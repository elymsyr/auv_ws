# Generic command message for subsystems
builtin_interfaces/Time timestamp

uint8 system_id      # Enum mapping to SystemID. 7 for 'all systems'.
uint8 command_code   # Enum for the specific command (e.g., INIT, START)
