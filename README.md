# Imperial College EEE - ELEC60013 Embedded Systems

## Group MoonHack - Music synthesiser

This document contains the project documentation, following the specifications required for the report. The contents are structured as follows:

## Table of Contents

1. [Project Overview](#project-overview)
2. [Task Identification and Implementation](#task-identification-and-implementation)
3. [Shared Data Structures and Synchronization](#shared-data-structures-and-synchronization)
4. [Timing Analysis](#timing-analysis)

## Project Overview

A brief introduction to the project, its purpose, and the system's main components.

## Task Identification and Implementation

### Classes and Headers

We have created classes for knobs and Waves to simplify code organization, improves readability, and enables easy modification of functionality.

- **Knobs**:

  We created the **Knob** class to enhance code efficiency by encapsulating knob-related data and behavior into a single, reusable object. The class has private and public members, including the knob ID, previous knob state, and rotation value. It provides constructors for initializing a knob with or without a starting rotation value, a method to print the current rotation, and a decodeKnob method for processing the current knob state. 


- **Waves**:

  The Waves class was created to consolidate wave-related functionality into a single, reusable object. It contains public methods for generating sine and sawtooth waveforms, which accept a phase accumulator as input. The get_sine method computes the waveform based on the keyStr and RX_keyStr values, while the get_sawtooth method calculates the waveform directly from the phase accumulator.

### Task Descriptions and Code Structure:
Pursuing the concept of maintainable code, we have employed distinct threads for individual tasks. Our system comprises four threads and 1 interrupts, arranged according to their priority:

- **sampleISR**

  ```
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  ```

  The sampleISR() function operates as an interrupt-driven routine that produces a precisely timed output waveform, initiated 22,000 times per second. Utilising interrupts enables the accurate handling of time-sensitive tasks, such as waveform generation. This function is responsible for maintaining the frequency's phase, updating it, forming and adjusting the output voltage, and producing the analog signal. All tasks occur inside the ISR, activated by an interrupt, to guarantee faultless and exact execution. Separate threads are not needed for performing these duties. The interrupt consistently and accurately initiates the function, while a configured timer activates the interrupt and calls the sampleISR() function.
- **CAN_RX_ISR**

- **scanKeyTask**
  ```
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
    scanKeysTask,		/* Function that implements the task */
    "scanKeys",		/* Text name for the task */
    64,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    4,			/* Task priority */
    &scanKeysHandle );  /* Pointer to store the task handle */
  ```
  In the scanKeysTask routine within the main cycle, a local step size variable is employed, and its finished value is atomic-wisely transferred to currentStepSize once it is determined. This reduces international variable accession and eliminates potential synchronisation errors caused by sampleISR() reading of currentStepSize when it's still partly altered in the main loop.

  Furthermore, a thread has been implemented to facilitate the scanning of keys, allowing for the continual updating of the display at various intervals. The keyArray variable is identified both as volatile and global, effectively granting access by scanKeysTask() and the core loop. Unquestionably, scanKeysTask() works discreetly with level 4 (high priority) and adheres to a constant rate of 50ms via the RTOS vTaskDelayUntil() capacity. This procedure supplies the option to station the thread in a waiting state while giving the CPU permission to execute other operations until the commencement of the function's reoccurrence. Time-spanning a mere 50 ms further ensures accuracy when making rotary adjustments, as it multiplies the chance of capturing unstable states courtesy of intensified matrix scanning frequency.


- **displayUpdate**
  ```
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
    displayUpdateTask,		/* Function that implements the task */
    "displayUpdate",		/* Text name for the task */
    256,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    3,			/* Task priority */
    &displayUpdateHandle );  /* Pointer to store the task handle */
  ```

  The displayUpdateTask is employed to present the system's visual output. It runs in a never-ending cycle and pauses for a predefined duration specified by xFrequency through utilising vTaskDelayUntil before continuing on with the display update steps. This is verified to make sure that the tasks do not use up too much energy. 

  The presentation covers the ongoing condition of the system, such as the respective key or button being pushed (keyStr and RX_keyStr), its volume (volVar), active octave (OCTAVE), and whether it is in either master or slave mode (master).

- **CAN_TX_Task**
  ```
  TaskHandle_t decodeHandle = NULL;
  xTaskCreate(
    decodeTask,		/* Function that implements the task */
    "decode",		/* Text name for the task */
    32,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    2,			/* Task priority */
    &decodeHandle );  /* Pointer to store the task handle */
  ```

  We implemented a transmit queue in the CAN_TX() function to address its non-thread-safe nature and prevent it from getting stuck while waiting for bus availability. By adding a new transmit thread to read messages from the queue and place them in the outgoing mailbox, we guaranteed an effective deployment of CPU cycles and eliminated hardware polling. A semaphore was used to indicate message acceptance, given by an ISR when space is available in the outgoing mailbox and taken by the transmit thread before loading a message. This approach allows for queued message transmission without wasting CPU resources.


- **decodeTask**
  ```
  TaskHandle_t CAN_TXHandle = NULL;
  xTaskCreate(
    CAN_TX_Task,		/* Function that implements the task */
    "CAN_TX",		/* Text name for the task */
    32,      		/* Stack size in words, not bytes */
    NULL,			/* Parameter passed into the task */
    1,			/* Task priority */
    &CAN_TXHandle );  /* Pointer to store the task handle */
  ```
  We created a decode thread with a function called decodeTask to process messages in the queue. The thread is initiated based on data availability in the queue, not by a tick counter. The xQueueReceive() call blocks and yields the CPU to other tasks until a message is available, and it's placed within the infinite loop in the decode thread. We removed the CAN_RX() call from the display function and initialised the decode thread in the setup. The RX_Message array is converted to a global variable for access in both decode and display tasks as well.


### Advance Features:
- **Sinewave**

  To implement the sinewave, a lookup table (LUT) is created and populated with sine values. The sine_LUT() function calculates the sine values for each step in the LUT, which contains 128 entries. This LUT is later utilized in the get_sine() function to generate the sinewave based on the phase accumulator. The function calculates the sinewave's output (Vout) by iterating through the keyStr and RX_keyStr values, considering the current octave, and using the LUT to access the appropriate sine values.

- **Chord**

  The chord feature is incorporated into the sinewave implementation by adjusting the get_sine() function. The function processes both local and remote key presses (tempkeyVal and tempRXkeyVal) in separate loops, adding their respective sine values to the output Vout_zeroCount. This allows the creation of chords by combining the sinewave output of multiple key presses. The final output (Vout) is computed by scaling the Vout_zeroCount according to the volume variable (volVar).

- All the sound is played in one speaker



## Shared Data Structures and Synchronization
## Timing Analysis

| Thread      | Time (μs)    | Interrupts | Time (μs) |
|-------------|--------------|------------|-----------|
| ScanKey     | 249.09       |            |           |
| DisplayMon  | 16097.75     |            |           |
| DecodeTask  | 9.5          |            |           |
| CAN_TX_Task | 12.25        |            |           |

### CPU Utilisation
