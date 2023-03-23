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
Pursuing the concept of maintainable code, we have employed distinct threads for individual tasks. Our system comprises 3 interrupts and 4 threads, arranged according to their priority:

| **Name**                                        | **Type**   |**Priority**|
|-------------------------------------------------|------------|------------|
| [sampleISR](#sampleisr)                         | Interrupt  | Highest    |
| [CAN_RX_ISR](#can_rx_isr)                       | Interrupt  | Highest    |
| [CAN_TX_ISR](#can_tx_isr)                       | Interrupt  | Highest    |
| [scanKeyTask](#scankeytask)                     | Thread     | 4          |
| [displayUpdate](#displayUpdate)                 | Thread     | 3          |
| [decodeTask](#decodeTask)                       | Thread     | 2          |
| [CAN_TX_Task](#can_tx_task)                     | Thread     | 1          |


- #### **sampleISR**

  ```
  HardwareTimer *sampleTimer = new HardwareTimer(Instance);
  sampleTimer->setOverflow(22000, HERTZ_FORMAT);
  sampleTimer->attachInterrupt(sampleISR);
  sampleTimer->resume();
  ```

  The sampleISR() function operates as an interrupt-driven routine that produces a precisely timed output waveform, initiated 22,000 times per second. Utilising interrupts enables the accurate handling of time-sensitive tasks, such as waveform generation. This function is responsible for maintaining the frequency's phase, updating it, forming and adjusting the output voltage, and producing the analog signal. All tasks occur inside the ISR, activated by an interrupt, to guarantee faultless and exact execution. Separate threads are not needed for performing these duties. The interrupt consistently and accurately initiates the function, while a configured timer activates the interrupt and calls the sampleISR() function.
- #### **CAN_RX_ISR**
  ```
  void CAN_RX_ISR (void) {
    uint8_t RX_Message_ISR[8];
    uint32_t ID = 0x123;
    CAN_RX(ID, RX_Message_ISR);
    xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
  }
  ```
  This interrupt is defined to handle the reception of data from the CAN bus and triggered when data is available on the CAN bus, ensuring timely and efficient processing of incoming messages.

  In the implementation of CAN_RX_ISR, the interrupt is triggered when a new message is available on the CAN bus. Upon activation, the ISR efficiently extracts the message data and transfers it to a message input queue (msgInQ) using the xQueueSendFromISR function.By utilizing interrupts, the system can react immediately to the availability of the RX_message, leading to improved performance and responsiveness.

- #### **CAN_TX_ISR**
  ```
  void CAN_TX_ISR (void) {
    xSemaphoreGiveFromISR(CAN_TX_Semaphore, NULL);
  }
  ```

  Utilizing interrupts in this context enables the system to react promptly when the CAN bus is ready to accept new data, ensuring smooth and timely data transmission.


- #### **scanKeyTask**
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

  Furthermore, a thread has been implemented to facilitate the scanning of keys, allowing for the continual updating of the display at various intervals. The keyArray variable is identified both as volatile and global, effectively granting access by scanKeysTask() and the core loop. Unquestionably, scanKeysTask() works discreetly with level 4 (high priority) and adheres to a constant rate of 20ms via the RTOS vTaskDelayUntil() capacity. This procedure supplies the option to station the thread in a waiting state while giving the CPU permission to execute other operations until the commencement of the function's reoccurrence. Time-spanning a mere 20 ms further ensures accuracy when making rotary adjustments, as it multiplies the chance of capturing unstable states courtesy of intensified matrix scanning frequency.


- #### **displayUpdate**
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

- #### **decodeTask**

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
  We created a decode thread with a function called decodeTask to process messages in the queue. The thread is initiated based on data availability in the queue, not by a tick counter. The xQueueReceive() call blocks and yields the CPU to other tasks until a message is available, and it's placed within the infinite loop in the decode thread. We removed the CAN_RX() call from the display function and initialised the decode thread in the setup. The RX_Message array is converted to a global variable for access in both decode and display tasks as well.

- #### **CAN_TX_Task**
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

  We implemented a transmit queue in the CAN_TX() function to address its non-thread-safe nature and prevent it from getting stuck while waiting for bus availability. By adding a new transmit thread to read messages from the queue and place them in the outgoing mailbox, we guaranteed an effective deployment of CPU cycles and eliminated hardware polling. A semaphore was used to indicate message acceptance, given by an ISR when space is available in the outgoing mailbox and taken by the transmit thread before loading a message. This approach allows for queued message transmission without wasting CPU resources.


### Advance Features:
- **Sine Wave**

  To implement the sinewave, a lookup table (LUT) is created and populated with sine values. The sine_LUT() function calculates the sine values for each step in the LUT, which contains 128 entries. This LUT is later utilized in the get_sine() function to generate the sinewave based on the phase accumulator. The function calculates the sinewave's output (Vout) by iterating through the keyStr and RX_keyStr values, considering the current octave, and using the LUT to access the appropriate sine values.

- **Chords**

  The chord feature is incorporated into the sinewave implementation by adjusting the get_sine() function. The function processes both local and remote key presses (tempkeyVal and tempRXkeyVal) in separate loops, adding their respective sine values to the output Vout_zeroCount. This allows the creation of chords by combining the sinewave output of multiple key presses. The final output (Vout) is computed by scaling the Vout_zeroCount according to the volume variable (volVar).

- **All the sound is played in one speaker**

  One of the keyboards acts as the `master` of the whole system. The other keyboards act as its `slaves` and transmit messages to the `master` to operate, which means that all the notes that are supposed to be played in the `slave` keyboard ended up playing through the `master` keyboard. RX_Message recevied by the `master` includes the corresponding keyStrArray for the transmitting keyboard (slave).

- **Auto Detection of master and slave**

  The allocation of master and slave is implemented for ease of access. It allows the users to change the speaker which the sound is playing from. The difference between the configuration for master and slave, is that the master is considered as the reciever and the slaves are considered as the transmitters. To implement this, WEST DETECT and EAST DETECT are used, depending on the DETECT bit, a bool variable `master` is set to `true` or `false`. The leftmost keyboard is considered to be the master, anything to its left is considered as the slave. When `master` is set to `true`, the keyboard is able to recieve messages but it will not transmit messages. When `master` is set to `false`, the keyboard will transmit messages and will not play any notes, this is because the master speaker is used to play all the notes.


## Shared Data Structures and Synchronization
## Timing Analysis
A timing analysis was needed to evaluate the system. A worst case was determined for each task to make sure that timings were accurate and all scenarios were considered. For scanKeyTask, the worse case scenario is where all keys are being pressed at the same time. For displayUpdateTask, the worse case scenario is where everything is being displayed (eg. All notes are played and displayed on the screen)

To ensure that 12 messages can be processed within 20ms, the CAN_TX_Task() function needs to complete one iteration every 1.67ms on average. However, this is a very short initiation interval for a threaded task and requires a high priority. The task doesn't need to process a message every 1.67ms because the messages are buffered by the queue. In fact, a queue of length 36 would take some time to fill up and the task needs to initiate at least once during this time so it can loop through all the queued messages. Therefore, the correct initiation interval is 60ms for 36 executions.

Similarly for the decode task, it also governed by a 36-item queue. The minimum transmission time of the CAN frames is 0.7ms so, in the worst case, the queue could fill in 25.2ms. That means the analysis for this task should be based on 36 executions with an initiation interval of 25.2ms.

| Threads                   | Priority  | Initiation Time (ms) | Execution Time (μs) | $$(\frac{\tau_n}{\tau_i}) \cdot T_i$$ (ms) | CPU Utilization (%) |
|---------------------------|-----------|----------------------|---------------------|--------------------------------------------|---------------------|
| scanKeyTask               |         4 |                   20 |              249.09 |                                      0.747 |                1.25 |
| displayUpdateTask         |         3 |                   50 |            16097.75 |                                       19.3 |                32.2 |
| decodeTask                |         2 |                 25.2 |                 9.5 |                                      0.023 |               0.038 |
| CAN_TX_Task               |         1 |     60 (for 36 exec) |               12.25 |                                       12.3 |               0.020 |
| sampleISR (Sine Wave)     | Interrupt |                0.045 |                9.09 |                                       12.1 |                20.2 |
| sampleISR (Sawtooth Wave) | Interrupt |                0.045 |                9.16 |                                       12.2 |                20.4 |
| Total                     |           |                      |                     |                                      56.67 |                74.3 |

The total latency is 56.67 ms which is less than the initiation time of the task with lowest priority.



## Critical instant analysis 
