# Pi ↔ Arduino Command Protocol

**Pi → Arduino:**
- "FORWARD [speed]" → Move motors forward
- "REVERSE [speed]" → Move motors backward
- "STOP" → Stop motors

**Arduino → Pi:**
- "ACK [command]" → Confirmed receipt
- "ERR [reason]" → Error message
- "STATUS [motor_state]" → Send current motor status
    - "Motor Forward"
    - "Motor Stop"
    - "Arduino Ready"