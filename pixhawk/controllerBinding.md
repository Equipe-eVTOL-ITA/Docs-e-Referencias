# Binding a Radio Receiver to a FlySky FS-i6X Controller

This is a quick guide on how to bind a radio receiver (connected to a Pixhawk) to a FlySky FS-i6X transmitter and configure the correct protocol.

---

## Receiver  
1. Power on the receiver **while holding the "Bind" button**.  
   - This puts it into bind mode, indicated by a rapidly flashing LED.  
2. Some receiver models do not have a bind button. Instead, they come with a **bind plug** that shorts the **signal** and **ground** pins to force bind mode.  

---

## Transmitter  
1. With the receiver already in bind mode, turn on the FlySky FS-i6X **while holding the "Bind" button**.  
2. The transmitter and receiver will bind automatically.  
3. After binding, power-cycle both the transmitter and receiver (turn them off and back on, without the bind plug).  

---

## Checking  
- After successful binding, the receiver’s LED should stay **solid** (not flashing).  
- In **QGroundControl**, go to **Vehicle Setup → Radio**.  
- Move the sticks and switches on the transmitter — you should see the channel bars respond.  

