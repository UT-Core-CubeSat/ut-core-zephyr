See <u>Files</u> above for function documentation.

Back to [Contents](@ref index)

# COMMS - Communications firmware  

Comms is the UT-CORE CubeSat's communications board, running Zephyr on an STM32U5 and communicating with CDH over CAN. 



# Software Requirements (Cason 10-9-26)
- Transmit and receive data 
    - **Incomplete** The eventual plan is to have the COMMS board communicating with the ground station via UHF, and sending payload images via S-Band. From my research, we'll probably have to decide on a format of messages. We could look into AX.25 or something similar for inspiration or implementation. 
- Collect data frames from uplink 
    - **Incomplete** The sofware will have to deserialize the data frames into something usable. It will also need to be handle partial/dropped broadcasts and store that data.   
- Collect data frames from CDH output to RF 
    - **Incomplete** When complete, CDH will send data frames containing telemetry, SOH, and other information about the CubeSAT. This board will need to serialize those and transmit them via UHF.
- Package SOH data and send to CDH 
    - **Incomplete** This should be a very similar implementation to what's already done in GNSS and ADCS. 
- Collect data from Payload board
    - **Incomplete** The current design is for a thermal camera to be the payload, but that may not always be the case for this software. Whatever it ends up being in the future, the goal is the same. Take the data from Payload and transmit it via S-Band. It will have to serialize that data first. 
- State Machine
    - **Incomplete** COMMS will need to be behave based on the state machine of the overall system. This will be transmitted from CDH via CAN.
- Beacon
    - **Incomplete** I'm not as sure on this one, but we might want to set up some kind of basic beacon info being transmitted periodically. I know many small satellites do something similar, but I don't know if ours is required to do this.
- ACK / Retransmission
    - **Incomplete** In the case of partial/dropped broadcasts, we'll need some kind of "Didn't get last message, retransmit please" built-in. We should also send an ACK to the ground station in the event of success on a command. 

# Important things to decide in the future
- Firmware updates
    - Will we be doing over-air firmware updates? In theory using MCUboot and loading the firmware into one core, booting from that core, and loading the firmware into the secondary core is possible. But if something goes wrong, it could mean mission failure. If we do decide to go this path, we'll have to implement image validation via hashing, as well as some kind of fallback in case of failure.
- Security
    - We'll need some kind of security setup for ensuring only authorized commands get interpreted. My intial suggestion is to use something like HMAC, which is standard for smaller projects like this and is secure enough. We'll also need to implement some kind of replay protection, so some hypothetical malicious party can't record our messages and send them again later. Could be a sequence number, or some kind of timestamp combined with a tolerance window.
- Image Buffering
    - How do we want to handle buffering images between receipt from payload to transmission? Do we store them to FRAM, or do we have some kind of external flash memory on the boards? And when we compress the images, do we want to have lossless compression and store fewer images, or lossy and store more?
- Priority
    - With this board sharing UHF and S-BAND, we'll have to decide which one is more important. Once that decision is eventually made, the actual implementation of that should be fairly simple. CDH utilizes threads with priority, and we could have at least two threads, one for each antanna. Have a state change to "transmitting" that the S-BAND thread checks for, and at that point it turns on. If we decide to add a beacon, that can also be its own thread, almost certainly at the lowest priority. 