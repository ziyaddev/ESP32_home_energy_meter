/*----------------------------------------------------
Modbus registers for SOCOMEC COUNTIS E13 ENERGY METER
----------------------------------------------------*/

#define REG_SOCO_VOLTAGE 50520              // Words : 2 | Simple voltage : V1 | Unit : V / 100

#define REG_SOCO_CURRENT 50528              // Words : 2 | Current : I1 | Unit : A / 1000

#define REG_SOCO_FREQ 50526                 // Words : 2 | Frequency : F | Unit : Hz / 1000

#define REG_SOCO_ACTIV_POWER 50544          // Words : 2 | Active Power phase 1 +/- : P1 | Unit : W / 0.1

#define REG_SOCO_REACT_POWER 50550          // Words : 2 | Reactive Power phase 1 +/- : Q1 | Unit : var / 0.1

#define REG_SOCO_APPAR_POWER 50556          // Words : 2 | Apparent Power phase 1 : S1 | Unit : VA / 0.1

#define REG_SOCO_SUM_POWER_FACTOR 50542     // Words : 2 | Sum Power Factor : -: leading et + : lagging : PF | Unit : - / 1000
#define REG_SOCO_PH1_POWER_FACTOR 50562     // Words : 2 | Power Factor phase 1 -: leading and + : lagging : PF1 | Unit : - / 1000

#define REG_SOCO_TOT_POS_ACT_ENERGY 50946   // Words : 2 | Total Positive Active Energy (no resetable) : Ea+ | Unit : Wh / 0.1
#define REG_SOCO_TOT_POS_REACT_ENERGY 50948 // Words : 2 | Total Positive Reactive Energy (no resetable) : Er+ | Unit : varh / 0.1
#define REG_SOCO_TOT_NEG_ACT_ENERGY 50952   // Words : 2 | Total Negative Active Energy (no resetable) : Ea- | Unit : Wh / 0.1
#define REG_SOCO_TOT_NEG_REACT_ENERGY 50954 // Words : 2 | Total Negative Reactive Energy (no resetable) : Er- | Unit : varh / 0.1
