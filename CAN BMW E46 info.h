/*
BMW E46 CAN Bus Information - Consolidated Reference
==================================================

CAN Bus Specifications:
- Speed: 500kb/s
- Data format: 8 bytes per message (Byte 0-7)
- Authors: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project/page1

IMPORTANT NOTES:
- This file consolidates all known CAN bus information for BMW E46
- Simplified format is provided first, followed by detailed MS43 specifications
- For calculations: LSB = Least Significant Byte, MSB = Most Significant Byte

================================================================================
                            SIMPLIFIED FORMAT (Legacy)
================================================================================

ARBID: 0x153 (ASC1) - Speed
-B1-B2: Speed [LSB-MSB] - 0x0008 = 1 km/hr

ARBID: 0x316 (DME1) - Engine RPM
-B2-B3: RPM [LSB-MSB] - RPM = (hex2dec("byte3"&"byte2"))/6.4

ARBID: 0x329 (DME2) - Engine Temperature & Cruise Control
-B1: Engine Temp - Temp in °C = .75 * hex2dec(byte01) - 48.373
-B3: Cruise Control (bit0 = LSB)
  bit 7: Cruise 1/0
  bit 6: Cruise -
  bit 5: Cruise +
  bit 6&5: Cruise Resume
-B5: Throttle position (00-FE)
-B6: Kickdown switch depressed = 4, Brake pedal depressed = 1

ARBID: 0x338 (DME3) - Sport Mode (MS45/MSV80 only)
-B2: Sport Mode Status
  0 = Sport on (request by SMG transmission)
  1 = Sport off
  2 = Sport on
  3 = Sport error

ARBID: 0x43F - Automatic Gear Display
-B0: 0x81 (no effect on IKE)
-B1: Gear Position
  01=1st, 02=2nd, 03=3rd, 04=4th, 05=D, 06=N, 07=R, 08=P, 09=5, 0A=6
-B2: Mode Display
  FF=no display, 00=E, 39=M, 40=S
-B5: 0x80 clears gear warning, other values activate it

ARBID: 0x545 (DME4) - Dashboard Warning Lights & Oil Temperature
-B0: Warning lights (can combine)
  bit 1: Check engine light
  bit 3: Cruise light
  bit 4: EML light
  bit 6: Check gas cap (2002+)
-B1-B2: Fuel consumption [LSB-MSB] (rate of change)
-B3: Additional lights
  bit 0: Oil level (2002+)
  bit 3: Overheat light
  M3 specific: 0x10=7K+ RPM, 0x20=6.5K+ RPM, 0x40=5.5K+ RPM
-B4: Oil Temperature - Temp in °C = hex2dec(byte04) - 48.373
-B5: bit 0 = Battery charge light (2002+)
-B7: bit 7 = Oil pressure light

ARBID: 0x610 (ICL1) - VIN Information (Remote Frame)
This contains last 7 digits of VIN. Sent by instrument cluster when requested by ABS.
-B0: Last VIN digit (drop trailing zero)
-B1: 3rd & 2nd VIN digits from end (hex)
-B2: 5th & 4th VIN digits from end (hex)
-B3: 6th VIN digit from end (ASCII)
-B4: 7th VIN digit from end (ASCII)
-B5-B7: 0
Example: B4=4B(K), B3=43(C), B2=71, B1=80, B0=90 → VIN: KC17809

ARBID: 0x613 - Odometer & Fuel Level (sent after 0x615 acknowledged)
-B0-B1: Odometer [LSB-MSB] - Convert to decimal, multiply by 10 = km
-B2: Fuel level - Full=0x39, Light on=0x08, Empty range=0x87 to 0x80
-B3-B4: Running clock [LSB-MSB] - Minutes since battery disconnect

ARBID: 0x615 - Climate Control & Outside Temperature
-B0: AC signal - 0x80 when on
-B3: Outside air temperature - Range: -40°C to +50°C
  Formula: (x>=0) ? DEC2HEX(x) : DEC2HEX(-x)+128

ARBID: 0x1F0 (ASC2/ABS) - Wheel Speeds
Individual wheel speeds (12-bit each, Intel LSB, unsigned, gain 1/16):
- Wheel 1: (B0 + (B1 & 0x0F)*256) / 16 = km/h
- Wheel 2: (B2 + (B3 & 0x0F)*256) / 16 = km/h  
- Wheel 3: (B4 + (B5 & 0x0F)*256) / 16 = km/h
- Wheel 4: (B6 + (B7 & 0x0F)*256) / 16 = km/h
(Multiply by 0.621 for MPH)

ARBID: 0x1F5 - Steering Angle Sensor
-B0-B1: Steering angle (15-bit, Intel LSB, gain 0.045°, sign bit 15)
-B2-B3: Angular velocity (15-bit, Intel LSB, gain 0.045°/s, sign bit 31)

ARBID: 0x1F8 (ASC4) - Brake Pressure & DSC
-B2: Brake pressure (1 bar per unit)

================================================================================
                        DETAILED MS43 SPECIFICATIONS
================================================================================
DME1 0x316 - Engine Management (Detailed)
Refresh Rate: 10ms

Byte 0 - Status Bitfield
  Bit 0: LV_SWI_IGK - Ignition Key Voltage (0=off, 1=on)
  Bit 1: LV_F_N_ENG - Crankshaft Sensor (0=ok, 1=error)
  Bit 2: LV_ACK_TCS - Traction Control State
  Bit 3: LV_ERR_GC - Gear Change Error (0=not possible, 1=possible)
  Bit 4-5: SF_TQD - Charge Intervention State (0-3)
  Bit 6: Unused
  Bit 7: LV_F_SUB_TQI - MAF Error (0=ok, 1=error)

Byte 1: TQI_TQR_CAN - Indexed Engine Torque (% of C_TQ_STND)
  Calculation: HEX × 0.390625

Byte 2-3: N_ENG - Engine Speed [LSB-MSB]
  Calculation: ((MSB × 256) + LSB) × 0.15625 = RPM

Byte 4: TQI_CAN - Indicated Engine Torque (% of C_TQ_STND)
  Calculation: HEX × 0.390625

Byte 5: TQ_LOSS_CAN - Engine Torque Loss (% of C_TQ_STND)
  Calculation: HEX × 0.390625

Byte 6: Error Bitfield
  Bit 6-7: ERR_AMT_CAN

Byte 7: TQI_MAF_CAN - Theoretical Engine Torque (% of C_TQ_STND)
  Calculation: HEX × 0.390625

DME2 0x329 - Engine Parameters (Detailed)
Refresh Rate: 10ms

Byte 0 - Multiplexed Information
  Bit 0-5: MUX_INFO
    CAN_LEVEL = CAN bus function level (always 0x11 for MS43)
    OBD_STEUER = GS diagnosis status for gearbox feedback
    MD_NORM = Refactored C_TQ_STND (0x3FF→0x3F, 16Nm resolution)
  Bit 6-7: MUX_CODE (0=CAN_LEVEL, 2=OBD_STEUER, 3=MD_NORM)

Byte 1: TEMP_ENG - Engine Coolant Temperature
  Calculation: (HEX × 0.75) - 48°C
  Range: 0x01 (-48°C) to 0xFF (142.5°C)

Byte 2: AMP_CAN - Ambient Pressure
  Calculation: (HEX × 2) + 598 hPa
  Range: 0x01 (600hPa) to 0xFE (1106hPa), 0xFF=Error

Byte 3 - Status Bitfield
  Bit 0: LV_SWI_CLU - Clutch Switch (0=released, 1=pressed)
  Bit 1: LV_LEVEL_IS - Idle Regulator (0=above, 1=below threshold)
  Bit 2: LV_ACK_CRU_AD_ECU - ACC1 Message ACK
  Bit 3: LV_ERU_CAN - Engine Running (0=stopped, 1=running)
  Bit 4: STATE_CRU_CAN
  Bit 5-7: STATE_MSW_CAN - Cruise Control Buttons
    0=None, 1=Set/Accel, 2=Decel, 3=Resume, 4=Deactivate, 7=Error

Byte 4: TPS_VIRT_CRU_CAN - Virtual Throttle Position
  Calculation: HEX × 0.390625

Byte 5: TPS_CAN - Accelerator Pedal Position (% of PVS_MAX)
  Calculation: HEX × 0.390625
  Range: 0x01 (0%) to 0xFE (99.2%), 0xFF=Error

Byte 6 - Input Status Bitfield
  Bit 0: LV_BS - Brake Switch (0=not pressed, 1=pressed)
  Bit 1: LV_ERR_BS - Brake Switch Error (0=ok, 1=faulty)
  Bit 2: LV_KD_CAN - Kick Down (0=inactive, 1=active)
  Bit 3-5: STATE_CRU_CAN - Cruise Control State
  Bit 6-7: REQ_SHIFTLOCK - Shift Lock Request

Byte 7: Unused

DME3 0x338 - Sport Mode (MS45/MSV80 only)
Refresh Rate: 1000ms and at signal change

Byte 0-1: Unused
Byte 2: STATE_SOF_CAN - Sport Button Status
  0=On (SMG request), 1=Off, 2=On, 3=Error, Init=0x01
Byte 3-7: Unused

DME4 0x545 - Warning Lights & Fuel (Detailed)
Refresh Rate: 10ms

Byte 0 - Warning Light Bitfield
  Bit 1: LV_MIL - Check Engine Light
  Bit 3: LV_MAIN_SWI_CRU - Cruise Main Switch
  Bit 4: LV_ETC_DIAG - EML Light
  Bit 6: LV_FUC_CAN - Fuel Tank Cap Light

Byte 1-2: FCO - Fuel Consumption [LSB-MSB]

Byte 3 - Additional Warning Bitfield
  Bit 0-2: Oil Level Indicator (consumption/loss/sensor error)
  Bit 3: LV_TEMP_ENG - Coolant Overheating Light
  Bit 4-6: M-Cluster Warm-Up LEDs
  Bit 7: Upshift Indicator

Byte 4: TOIL_CAN - Oil Temperature
  Calculation: HEX - 48°C
  Range: 0x00 (-48°C) to 0xFE (206°C)

Byte 5 - Additional Status
  Bit 0: Battery Charge Light (Alpina Roadster only)
  Bit 1-6: Unused

Byte 6: Oil Level (MSS54HP only)
  Calculation: (HEX - 158) / 10L
  Range: 0x80/0xC0 (-3.0L) to 0xBE/0xFE (+3.2L)

Byte 7 - Oil/Tire Status
  Bit 0: Tire Pressure State (MSS54 only)
  Bit 7: Engine Oil Pressure Low

ASC2 0x1F0 - Wheel Speed Sensors (Detailed)
Refresh Rate: 10ms (ASC) / 20ms (DSC)

Each wheel speed calculation: ((MSB × 256 + LSB) - 44) / 15.875 = km/h
Minimum value: 0x2C (0 km/h)

Byte 0-1: Wheel 1 Speed [LSB-MSB]
Byte 2-3: Wheel 2 Speed [LSB-MSB]
Byte 4-5: Wheel 3 Speed [LSB-MSB]
Byte 6-7: Wheel 4 Speed [LSB-MSB]

ASC4 0x1F8 - DSC Control & Brake Pressure (Detailed)
Refresh Rate: 20ms

Byte 0: S_WHEEL_ACC - Wheel Acceleration

Byte 1 - Control Status Bitfield
  Bit 0-2: S_HDC - Hill Descent Control Status
    00h=off, 01h=wrong gear, 02h=wrong range, 03h=excess speed,
    04h=temp inactive, 05h=enabled not active, 06h=active, 07h=error
  Bit 3: L_HDC
  Bit 4: Always 00h
  Bit 5: B_TW_MSR
  Bit 6: B_TW_ASC
  Bit 7: B_Offroad

Byte 2: P_BRAKE - Brake Pressure
  Calculation: HEX × 1 bar
  Range: 0x00 (0 bar) to 0xFF (255 bar)

Byte 3: RDR Bitfield (wheel-specific data)
  Bit 0-1: RDR_VL, Bit 2-3: RDR_VR
  Bit 4-5: RDR_HL, Bit 6-7: RDR_HR

Byte 4-5: TW_IND_ASR [LSB-MSB]
Byte 6-7: TW_IND_MSR [LSB-MSB]

ICL1 0x610 - VIN Information (Remote Frame - Detailed)
Sent by instrument cluster when requested by DSC for VIN integrity check.

Byte 0: Last VIN digit (drop trailing zero)
Byte 1: 3rd & 2nd VIN digits from end
Byte 2: 5th & 4th VIN digits from end  
Byte 3: 6th VIN digit from end (ASCII)
Byte 4: 7th VIN digit from end (ASCII)
Byte 5-7: Unused (0x00)

Example: B4=0x4B(K), B3=0x43(C), B2=0x71, B1=0x80, B0=0x90 → VIN: KC17809

================================================================================
                               NOTES FOR DEVELOPERS
================================================================================

1. Legacy format provides simple implementation examples
2. Detailed MS43 format provides complete protocol specifications
3. All temperature calculations use offset formulas
4. Bitfields allow multiple status indicators per byte
5. Multi-byte values typically use Intel LSB format (LSB first)
6. Refresh rates indicate how frequently messages are transmitted
7. Some messages are conditional (remote frames, signal changes)

*/