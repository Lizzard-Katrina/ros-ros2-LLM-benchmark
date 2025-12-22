# Task 010: Microphone Array → Sound Localization (HARK)

## Expected outcome for each TODO
1. **Constructor TODO (constructor_init.cpp)**:
   - Initialize audio buffers and processing state
   - Load algorithm parameters (sample rate, buffer size)
   - Prepare any processing modules or FFT buffers
2. **Audio processing TODO (audio_processing.cpp)**:
   - Perform FFT or HARK beamforming
   - Estimate sound source azimuth/elevation
   - Update estimatedDirection_ and publish to `/sound_direction`

## Validation
- Each TODO can be validated independently:
    - Constructor: node compiles, subscribers/publishers initialized
    - Audio processing: feed mock audio buffer, verify `/sound_direction` topic updates
- Full validation: use recorded multi-channel audio or simulated microphone array data
