Vehicle interface failure analysis {#vehicle-interface-failure-analysis}
=======================================================================

@tableofcontents

The purpose of this document is to identify ways in which the vehicle interface component
might fail, and to determine mitigation strategies for these failure modalities.


# Application logic

The purpose of the vehicle interface is to perform input validation on control inputs,
pass the result to the vehicle, and report information from the vehicle.

The general process for this application is then as follows:

1. Receive data
2. Timeout -> (optional) hazard lights and slow down
3. (Optional) compute acceleration/steer commands via controller
4. (Optional) low pass filter on controls
5. Read state commands
6. (Optional) pass commands through the state machine
7. Send final commands to the vehicle
8. Read information from vehicle
9. Publish information that's been read from vehicle
10. (Optional) update the state machine


# Failure modalities

For each of the steps, the following failure modes have been identified:

1. Receive data
  1. Input data is wrong
  2. Data from DDS/ADAS stack doesn't arrive in time
  3. Data is high frequency
  4. Data could be out of order
  5. Data is out of range
2. Timeout -> (optional) hazard lights and slow down
3. (Optional) compute acceleration/steer commands via controller
4. (Optional) low pass filter on controls
  1. Implementation is wrong (of the state machine, filter, controller, etc.)
  2. Filter could be not working right
5. Read state commands
  1. No state commands available (via read)
6. (Optional) pass commands through the state machine
  1. Conflicting commands are sent
7. Send final commands to vehicle
  1. Manual control might happen during autonomous control
  2. Commands might not be acknowledged by vehicle
  3. Command might not be executed by vehicle
8. Read information from vehicle
  1. Data from the vehicle platform doesn't arrive in time
  2. Sensor data from vehicle might be wrong or corrupted
10. (Optional) update state machine
  1. State machine doesn't get updated, or misses an update (putting it in an inconsistent state)


# Mitigations

For each failure modality, the following mitigations (or rationales on why mitigations are not
required) have been identified:

1. Input data is wrong
  1. Validate inputs via the state machine, ensure system integrity via security features
2. Data from DDS/ADAS stack doesn't arrive in time
  1. Come to a safe stop on data timeout
3. Data is high frequency
  1. Low pass filter
4. Data could be out of order
  1. Keep track of latest timestamp, ignore (and warn on) data that's older than the latest
  timestamp
5. Data is out of range
  1. Clamp values, using configured limits
  2. If data is far out of range, issue a warning; could be user error or some other issue
6. Implementation is wrong (of the state machine, filter, controller, etc.)
  1. Fully test implementations
7. Filter could be not working right
  1. Can do extra validation on the output; FFT, keep track of derivatives, or keep track of variance
8. No state commands available (via read)
  1. This is not an error: there is nothing to do
9. Conflicting commands are sent
  1. State machine makes commands consistent
10. Manual control might happen during autonomous control
  1. If vehicle is not in autonomous mode, don't even try to send commands
11. Commands might not be acknowledged by the vehicle
  1. If communication mechanisms supports acknowledgements, warn if commands have not been
  acknowledged within some timeout
12. Command might not be executed by vehicle
  1. Can add extra state to `SafetyStateMachine`, might need to keep some history since updates might
  lag a little
13. Data from vehicle platform doesn't arrive in time
  1. Depends on the platform
  2. This could be a critical error (e.g. DBW is down)
  3. The developer should definitely be notified, and the platform should perform timeout behavior
  (but might not fully mitigate the risk)
14. Sensor data from vehicle might be wrong or corrupted
  1. Ensure that all components elsewhere in the stack validate their inputs; implementer of
  interface should use domain-specific knowledge to validate
15. The state machine doesn't get updated, or misses an update (putting it in an inconsistent state)
  1. Minimize path length in the state machine--ensure that the state machine relies on last
  observation rather than a history of observations


# Summary

The mitigations proposed as a result of the failure analysis can be encoded with the following
architectural components:

1. Low pass filter
2. Safety state machine
  1. Data clamping can occur here
  2. Validating that data is low frequency can happen here
  3. Clamp control data to a safe range can occur here
  4. Warning when control data is wildly out of range can occur here
  5. State machine should have a short path length
  6. Warnings should be emitted if a state transition doesn't occur after being commanded (within
  some timeout)
  7. Combination of control and state commands should be made consistent here
3. Platform interface
  1. Commands should not be sent if vehicle is not in autonomous mode
  2. If data does not arrive in time from the platform, a warning or error should be raised
  3. If the vehicle communication mechanism supports acknowledgements, a warning should be raised
  if an acknowledgement was not received in time

And the following behaviors should be enforced by the overall implementation:

1. Safe stop on timeout
2. Ignore old data


# Related issues

- #4944: Add failure analysis document for vehicle interface
- #4770: Review new design articles for the 1.0.0 release
