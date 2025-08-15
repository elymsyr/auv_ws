# Generic state message for subsystems
builtin_interfaces/Time timestamp

uint8 system_id      # Enum mapping to SystemID
uint8 process_id     # Enum for process within the system
uint8 message_code   # Enum for the specific message
