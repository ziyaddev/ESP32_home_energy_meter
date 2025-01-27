/*----------------------------------------------------
Modbus registers for SOCOMEC COUNTIS E13 ENERGY METER
----------------------------------------------------*/

#define REG_SOCO_VOLTAGE 50520              // Phase 1 : phase voltage
#define REG_VOLTAGE_PH2 4098                // Phase 2 : phase voltage
#define REG_VOLTAGE_PH3 4100                // Phase 3 : phase voltage

#define REG_SOCO_CURRENT 50528              // Phase 1 : current

#define REG_SOCO_ACTIV_POWER 50544          // Phase 1 : active power

#define REG_ACTIV_POWER_PH1_SIGN 4146       // Phase 1 : sign of active power
#define REG_ACTIV_POWER_PH2_SIGN 4147       // Phase 2 : sign of active power
#define REG_ACTIV_POWER_PH3_SIGN 4158       // Phase 3 : sign of active power
#define REG_ACTIV_POWER_TOTAL 4116          // 3-phase : active power
#define REG_ACTIV_POWER_TOTAL_SIGN 4122     // 3-phase : sign of active power

#define REG_SOCO_REACT_POWER 50550          // Phase 1 : reactive power

#define REG_REACT_POWER_PH1_SIGN 4155       // Phase 1 : sign of reactive power
#define REG_REACT_POWER_PH2_SIGN 4156       // Phase 2 : sign of reactive power
#define REG_REACT_POWER_PH3_SIGN 4157       // Phase 3 : sign of reactive power
#define REG_REACT_POWER_TOTAL 4118          // 3-phase : reactive power
#define REG_REACT_POWER_TOTAL_SIGN 4123     // 3-phase : sign of reactive power

#define REG_SOCO_APPAR_POWER 50556          // 3-phase : apparent power

#define REG_SOCO_POWER_FACTOR 50562         // Phase 1 : power factor

#define REG_POWER_FACTOR_PH1_DOMAIN 4167    // 3-phase : sector of power factor (cap or ind) => 0="PF=1", 1="ind" (L), 2="cap" (C) 
#define REG_POWER_FACTOR_PH2_DOMAIN 4168    // 3-phase : sector of power factor (cap or ind) => 0="PF=1", 1="ind" (L), 2="cap" (C) 
#define REG_POWER_FACTOR_PH3_DOMAIN 4169    // 3-phase : sector of power factor (cap or ind) => 0="PF=1", 1="ind" (L), 2="cap" (C) 
#define REG_POWER_FACTOR_TOTAL 4132         // 3-phase : power factor
#define REG_POWER_FACTOR_TOTAL_DOMAIN 4133  // 3-phase : sector of power factor (cap or ind) => 0="PF=1", 1="ind" (L), 2="cap" (C) 

#define REG_SOCO_FREQ 50526                 // Frequency

#define REG_AVERAGE_POWER 4135              // 3-phase : average power

#define REG_RUN_HOUR 4206                   // Run hour meter in hours
#define REG_RUN_MINUTES 4220                // Run hour meter in minutes
#define REG_ACTIV_ENERGY_TOTAL 4224         // 3-phase : Total positive active energy
#define REG_REACT_ENERGY_TOTAL 4226         // 3-phase : Total positive reactive energy
