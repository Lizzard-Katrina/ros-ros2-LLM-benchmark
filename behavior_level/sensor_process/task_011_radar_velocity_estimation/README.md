# Task 011: Radar → Object Velocity Estimation

## Expected outcome for each TODO
1. **Constructor TODO (constructor_init.cpp)**:
   - Initialize radar data buffers and algorithm state
   - Load algorithm parameters (num beams, sample rate, max range)
   - Prepare FFT / Doppler processing modules
2. **Radar processing TODO (radar_processing.cpp)**:
   - Perform Doppler/FFT processing
   - Compute radial velocity for each detected object
   - Update radialVelocities_ and publish to `/radar_velocity`

## Validation
- Each TODO can be validated independently:
    - Constructor: node compiles, subscribers/publishers initialized
    - Radar processing: feed mock radar scan, verify `/radar_velocity` topic updates
- Full validation: use recorded radar data or simulated radar messages
