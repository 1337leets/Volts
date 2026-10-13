/* ================================================================
 * VOLTS — an electrical-engineering circuit puzzle for the terminal
 *
 * Build:  make            (or: gcc -std=c99 -Wall -Wextra -o volts volts.c -lncurses -lm)
 * Run:    ./volts
 *
 * Controls:
 *   A/B/C/D   : select a slot
 *   1-9, 0    : pick a part from the tray (assigns it to the selected slot)
 *   X         : clear the selected slot
 *   Enter     : submit / check the answer
 *   N         : next level (after solving the current one)
 *   Q         : quit
 *
 * Engineering model:
 *   DC  → Ohm's law, KVL/KCL, Thevenin
 *   AC  → complex impedance:  Z_R = R,  Z_C = 1/(jωC),  Z_L = jωL
 *   H(jω) = Vout/Vin — magnitude and phase are computed
 * ================================================================ */

#include <ncurses.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* ─── Color pairs ─── */
#define CP_TITLE   1
#define CP_SLOT    2
#define CP_SEL     3
#define CP_R       4
#define CP_C       5
#define CP_L       6
#define CP_LED     7
#define CP_OK      8
#define CP_ERR     9
#define CP_MEAS   10
#define CP_DIM    11
#define CP_WIRE   12
#define CP_BAR    13
#define CP_TRAY   14

/* ════════════════════════════════════════════════════════════════
 * COMPLEX ARITHMETIC
 *
 * In AC circuits impedance is complex:  Z = Re{Z} + j·Im{Z}
 *   - Re{Z}: resistive part (dissipates energy)
 *   - Im{Z}: reactive part  (stores energy)
 *
 * j = √−1,  j² = −1
 * ════════════════════════════════════════════════════════════════ */
typedef struct { double re, im; } Cx;

static Cx cx_add(Cx a, Cx b) {
    return (Cx){a.re + b.re, a.im + b.im};
}
static Cx cx_mul(Cx a, Cx b) {
    /* (a+jb)(c+jd) = (ac−bd) + j(ad+bc) */
    return (Cx){a.re*b.re - a.im*b.im,
                a.re*b.im + a.im*b.re};
}
static Cx cx_inv(Cx a) {
    /* 1/(a+jb) = (a−jb)/(a²+b²) */
    double d = a.re*a.re + a.im*a.im;
    if (d < 1e-30) return (Cx){0, 0};
    return (Cx){a.re/d, -a.im/d};
}
static Cx cx_div(Cx a, Cx b) { return cx_mul(a, cx_inv(b)); }
static double cx_mag(Cx a)   { return sqrt(a.re*a.re + a.im*a.im); }
static double cx_ang(Cx a)   { return atan2(a.im, a.re) * 180.0 / M_PI; }

/* Parallel impedance: Z1 || Z2 = Z1·Z2 / (Z1+Z2) */
static Cx cx_par(Cx z1, Cx z2) {
    return cx_div(cx_mul(z1, z2), cx_add(z1, z2));
}

/* ════════════════════════════════════════════════════════════════
 * COMPONENT TYPES & IMPEDANCE
 *
 * ω = 2πf  (angular frequency, rad/s)
 *
 * Resistor R:    Z_R = R          (purely real, phase = 0°)
 * Capacitor C:   Z_C = 1/(jωC)    (negative imaginary, phase = −90°)
 *                    = −j/(ωC)
 * Inductor L:    Z_L = jωL        (positive imaginary, phase = +90°)
 * DC (ω = 0):    C → open circuit (Z→∞),  L → short circuit (Z→0)
 * ════════════════════════════════════════════════════════════════ */
typedef enum { CT_NONE=0, CT_R, CT_C, CT_L, CT_LED, CT_WIRE } CompType;

typedef struct {
    CompType type;
    double   val;        /* in ohms, farads or henries */
    char     label[18];  /* display label */
} Comp;

static Cx impedance(CompType t, double val, double omega) {
    switch (t) {
        case CT_R:    return (Cx){val, 0};
        case CT_C:
            if (omega < 1e-10) return (Cx){1e9, 0};   /* DC → open circuit */
            return (Cx){0, -1.0/(omega * val)};
        case CT_L:
            return (Cx){0, omega * val};
        case CT_WIRE: return (Cx){1e-9, 0};            /* ideal wire */
        default:      return (Cx){1e9, 0};             /* open circuit */
    }
}

static const char *type_name(CompType t) {
    switch (t) {
        case CT_R: return "resistor";
        case CT_C: return "capacitor";
        case CT_L: return "inductor";
        default:   return "part";
    }
}

/* ════════════════════════════════════════════════════════════════
 * LEVEL DEFINITIONS
 * ════════════════════════════════════════════════════════════════ */
#define MAX_SLOTS  4
#define MAX_TRAY  10
#define N_LEVELS  10

typedef enum {
    TOPO_SERIES_DIV,      /* Voltage divider: Z1 on top, Z2 on bottom     */
    TOPO_LOAD_EFFECT,     /* Divider + fixed parallel load                */
    TOPO_CURRENT_DIV,     /* Two parallel branches, current divider       */
    TOPO_MAX_POWER,       /* Fixed Rth + adjustable R_load                */
    TOPO_LED_PROTECT,     /* LED + series protection resistor             */
    TOPO_RC_LOWPASS,      /* Series R + shunt C → low-pass                */
    TOPO_RC_HIGHPASS,     /* Series C + shunt R → high-pass               */
    TOPO_RLC_RESONANCE,   /* Series RLC → f0 target                       */
    TOPO_RLC_BANDPASS,    /* Series RLC → bandwidth target (R is chosen)  */
    TOPO_RC_PHASE,        /* RC low-pass → phase target                   */
} Topo;

typedef enum {
    GOAL_VOUT_EQ,         /* |Vout − p1| ≤ p2                             */
    GOAL_VOUT_GE,         /* Vout ≥ p1                                    */
    GOAL_CURRENT_RANGE,   /* p1 ≤ I_mA ≤ p2                               */
    GOAL_POWER_MAX,       /* P_mW ≥ p1                                    */
    GOAL_MAG_LE,          /* |H| ≤ p1                                     */
    GOAL_F0_RANGE,        /* p1 ≤ f0 ≤ p2   (Hz)                          */
    GOAL_BW_LE,           /* BW ≤ p1        (Hz)                          */
    GOAL_PHASE_EQ,        /* |φ − p1| ≤ p2  (degrees)                     */
} GoalType;

typedef struct {
    GoalType    type;
    double      p1, p2;   /* p1 = target / lower bound, p2 = tolerance / upper bound */
    const char *desc;
} Goal;

typedef struct {
    char        name;     /* 'A', 'B', 'C', 'D'                           */
    const char *role;     /* "Top resistor", "Shunt C", ...               */
    CompType    need;     /* required part type for this slot             */
} SlotDef;

typedef struct {
    const char *title;
    const char *concept;
    const char *theory;   /* short formula reminder                       */
    const char *hint;
    Topo        topo;
    double      Vs;       /* source amplitude                             */
    double      freq;     /* Hz  (0 = DC)                                 */
    double      fixed_val;/* topology-specific fixed value                */
    int         n_slots;
    SlotDef     slot_defs[MAX_SLOTS];
    int         n_tray;
    Comp        tray[MAX_TRAY];
    Goal        goal;
} LevelDef;

/* ════════════════════════════════════════════════════════════════
 * THE 10 LEVELS  (math checked against the formulas below)
 * ════════════════════════════════════════════════════════════════ */
static const LevelDef LEVELS[N_LEVELS] = {

/* ── 0: Voltage divider ────────────────────────────────────────
   Vout = Vs * R_B / (R_A + R_B)
   Target 4 V: R_A=200, R_B=100 → 12*(100/300) = 4.00 V ✓
   ────────────────────────────────────────────────────────────── */
{ "Voltage Divider", "KVL + Ohm's Law",
  "Vout = Vs * R_B / (R_A + R_B)",
  "You need a 1:2 ratio. Try R_A=200, R_B=100.",
  TOPO_SERIES_DIV, 12.0, 0, 0,
  2, {{'A',"Top resistor",CT_R}, {'B',"Bottom resistor",CT_R}},
  5, { {CT_R,100,"100 Ohm"}, {CT_R,200,"200 Ohm"}, {CT_R,470,"470 Ohm"},
       {CT_R,1000,"1k Ohm"}, {CT_R,2200,"2.2k Ohm"} },
  {GOAL_VOUT_EQ, 4.0, 0.5, "Vout = 4.0 V  (+-0.5)"}
},

/* ── 1: Load effect ────────────────────────────────────────────
   R_L = 100Ω is wired in parallel (fixed). R_B_eff = R_B || 100
   Vout = 9 * R_B_eff / (R_A + R_B_eff) >= 5.5 V
   R_A=47, R_B=470: R_eff=82.5 → 9*(82.5/129.5) = 5.73 V ✓
   ────────────────────────────────────────────────────────────── */
{ "Load Effect", "Thevenin's Theorem",
  "Effect of the R_L = 100 Ohm load: R_B_eff = R_B || R_L",
  "Pick a large R_B so the voltage doesn't sag under load.",
  TOPO_LOAD_EFFECT, 9.0, 0, 100.0,
  2, {{'A',"Series R_A",CT_R}, {'B',"Parallel R_B",CT_R}},
  5, { {CT_R,47,"47 Ohm"}, {CT_R,100,"100 Ohm"}, {CT_R,220,"220 Ohm"},
       {CT_R,470,"470 Ohm"}, {CT_R,1000,"1k Ohm"} },
  {GOAL_VOUT_GE, 5.5, 0, "Vout >= 5.5 V"}
},

/* ── 2: Current divider ────────────────────────────────────────
   Vs = 10 V is applied directly across each branch: I_A = Vs / R_A
   Target: I_A = 50 mA → R_A = 200Ω
   ────────────────────────────────────────────────────────────── */
{ "Current Divider", "KCL",
  "Parallel branches: I_A = Vs / R_A  (same voltage, current splits)",
  "For I_A = 50 mA: R_A = Vs/I = 10/0.05 = 200 Ohm",
  TOPO_CURRENT_DIV, 10.0, 0, 0,
  2, {{'A',"Branch-A R",CT_R}, {'B',"Branch-B R",CT_R}},
  5, { {CT_R,100,"100 Ohm"}, {CT_R,200,"200 Ohm"}, {CT_R,470,"470 Ohm"},
       {CT_R,1000,"1k Ohm"}, {CT_R,2200,"2.2k Ohm"} },
  {GOAL_CURRENT_RANGE, 45.0, 55.0, "I_A = 50 mA  (45-55 mA)"}
},

/* ── 3: Maximum power transfer ─────────────────────────────────
   Rth = 50Ω fixed.  P_load = (Vs/(Rth+R_load))² · R_load
   P_max = Vs²/(4·Rth) = 100/200 = 500 mW  at  R_load = Rth = 50Ω
   R=47:  P = 499.5 mW ✓ (≥ 450)      R=100: P = 444.4 mW ✗ (just short)
   R=22:  P = 424.4 mW ✗              R=220: P = 301.8 mW ✗
   ────────────────────────────────────────────────────────────── */
{ "Max Power Transfer", "Thevenin + Maximum Power Transfer",
  "P_max = Vs^2/(4*Rth), reached when R_load = Rth",
  "Rth = 50 Ohm. Pick the tray value closest to Rth.",
  TOPO_MAX_POWER, 10.0, 0, 50.0,
  1, {{'A',"R_load",CT_R}},
  5, { {CT_R,22,"22 Ohm"}, {CT_R,47,"47 Ohm"}, {CT_R,100,"100 Ohm"},
       {CT_R,220,"220 Ohm"}, {CT_R,470,"470 Ohm"} },
  {GOAL_POWER_MAX, 450.0, 0, "P_load >= 450 mW"}
},

/* ── 4: LED series resistor ────────────────────────────────────
   LED: Vf = 2.0 V.  I = (Vs−Vf)/R = 3/R
   5 mA ≤ I ≤ 15 mA → 200Ω ≤ R ≤ 600Ω
   R=220 → 13.6 mA ✓   R=330 → 9.1 mA ✓   R=470 → 6.4 mA ✓
   ────────────────────────────────────────────────────────────── */
{ "LED Driver", "Ohm's Law + Safe Operating Region",
  "I = (Vs - Vf) / R_series,  Vf = 2.0 V (LED forward voltage)",
  "I = 3 V / R  ->  5-15 mA means R between 200 and 600 Ohm.",
  TOPO_LED_PROTECT, 5.0, 0, 2.0,
  1, {{'A',"Series R",CT_R}},
  5, { {CT_R,100,"100 Ohm"}, {CT_R,220,"220 Ohm"}, {CT_R,330,"330 Ohm"},
       {CT_R,470,"470 Ohm"}, {CT_R,1000,"1k Ohm"} },
  {GOAL_CURRENT_RANGE, 5.0, 15.0, "5 mA <= I_LED <= 15 mA"}
},

/* ── 5: RC low-pass filter ─────────────────────────────────────
   H(jω) = Z_C/(Z_R+Z_C) = 1/(1+jωRC)
   |H| = 1/√(1+(ωRC)²)    fc = 1/(2πRC)
   Target: |H(1 kHz)| ≤ 0.30
   R=1k,  C=1µF: ωRC=6.28 → |H|=0.157 ✓
   R=470, C=1µF: ωRC=2.95 → |H|=0.321 ✗ (just outside)
   ────────────────────────────────────────────────────────────── */
{ "RC Low-Pass", "RC Filter, fc = 1/(2*pi*RC)",
  "|H(jw)| = 1/sqrt(1+(wRC)^2),  fc = 1/(2*pi*RC)",
  "For |H| <= 0.30 at 1 kHz, fc must be well below 1 kHz. Increase R*C.",
  TOPO_RC_LOWPASS, 1.0, 1000.0, 0,
  2, {{'A',"Series R",CT_R}, {'B',"Shunt C",CT_C}},
  7, { {CT_R,470,"470 Ohm"}, {CT_R,1000,"1k Ohm"}, {CT_R,4700,"4.7k Ohm"},
       {CT_C,10e-9,"10 nF"}, {CT_C,100e-9,"100 nF"},
       {CT_C,1e-6,"1 uF"},   {CT_C,10e-6,"10 uF"} },
  {GOAL_MAG_LE, 0.30, 0, "|H(1kHz)| <= 0.30"}
},

/* ── 6: RC high-pass filter ────────────────────────────────────
   H(jω) = jωRC/(1+jωRC)
   |H| = ωRC/√(1+(ωRC)²)
   Target: |H(100 Hz)| ≤ 0.20
   R=1k, C=100n: ωRC=0.0628 → |H|=0.0627 ✓
   R=1k, C=1µF:  ωRC=0.628  → |H|=0.532  ✗
   ────────────────────────────────────────────────────────────── */
{ "RC High-Pass", "RC Filter, Phase Characteristics",
  "|H(jw)| = wRC/sqrt(1+(wRC)^2),  fc = 1/(2*pi*RC)",
  "For |H| <= 0.20 at 100 Hz, fc must be well above 100 Hz. Decrease R*C.",
  TOPO_RC_HIGHPASS, 1.0, 100.0, 0,
  2, {{'A',"Series C",CT_C}, {'B',"Shunt R",CT_R}},
  7, { {CT_R,100,"100 Ohm"}, {CT_R,1000,"1k Ohm"}, {CT_R,10000,"10k Ohm"},
       {CT_C,10e-9,"10 nF"}, {CT_C,100e-9,"100 nF"},
       {CT_C,1e-6,"1 uF"},   {CT_C,10e-6,"10 uF"} },
  {GOAL_MAG_LE, 0.20, 0, "|H(100Hz)| <= 0.20"}
},

/* ── 7: Series RLC resonance ───────────────────────────────────
   f0 = 1/(2π√(LC)),  Q = ω0·L/R  (R = 100Ω fixed)
   Target: 4 kHz ≤ f0 ≤ 6 kHz
   L=1mH, C=1µF: f0 = 1/(2π√(10⁻³·10⁻⁶)) = 5.03 kHz ✓
   L=5mH, C=1µF: f0 = 2.25 kHz ✗
   ────────────────────────────────────────────────────────────── */
{ "Series RLC Resonance", "f0 = 1/(2*pi*sqrt(LC)),  Q = w0*L/R",
  "Resonance: Im(Z_total) = 0 -> wL = 1/(wC) -> w0 = 1/sqrt(LC)",
  "For f0 = 4-6 kHz: L = 1 mH, C = 1 uF -> f0 = 5.03 kHz.",
  TOPO_RLC_RESONANCE, 1.0, 0, 100.0,
  2, {{'A',"Inductor L",CT_L}, {'B',"Capacitor C",CT_C}},
  7, { {CT_L,1e-3,"1 mH"}, {CT_L,5e-3,"5 mH"}, {CT_L,10e-3,"10 mH"},
       {CT_C,47e-9,"47 nF"}, {CT_C,100e-9,"100 nF"},
       {CT_C,1e-6,"1 uF"},   {CT_C,10e-6,"10 uF"} },
  {GOAL_F0_RANGE, 4000.0, 6000.0, "4 kHz <= f0 <= 6 kHz"}
},

/* ── 8: RLC bandwidth ──────────────────────────────────────────
   L=1mH, C=1µF fixed → f0 = 5.03 kHz
   BW = R/(2πL)   Target: BW ≤ 2 kHz → R ≤ 12.57Ω
   R=10Ω: BW = 1.59 kHz ✓,  R=47Ω: BW = 7.48 kHz ✗
   ────────────────────────────────────────────────────────────── */
{ "RLC Bandwidth", "BW = R/(2*pi*L),  Q = f0/BW",
  "In a series RLC circuit the bandwidth is set by R",
  "L = 1 mH and C = 1 uF are fixed. BW <= 2 kHz needs R <= 12.6 Ohm -> R = 10 Ohm.",
  TOPO_RLC_BANDPASS, 1.0, 0, 0,
  1, {{'A',"Resistor R",CT_R}},
  5, { {CT_R,10,"10 Ohm"}, {CT_R,47,"47 Ohm"}, {CT_R,100,"100 Ohm"},
       {CT_R,220,"220 Ohm"}, {CT_R,470,"470 Ohm"} },
  {GOAL_BW_LE, 2000.0, 0, "BW <= 2.0 kHz"}
},

/* ── 9: RC phase shifter ───────────────────────────────────────
   φ = −arctan(ωRC)   (low-pass, measured across C)
   Target: φ = −45° ± 5°  at  f = 1 kHz → ωRC = 1 → RC = 159µs
   R=1k,   C=150n: RC=150µs, φ = −43.3° ✓
   R=1.5k, C=100n: RC=150µs, φ = −43.3° ✓
   R=2.2k, C=68n:  RC=150µs, φ = −43.2° ✓
   R=4.7k, C=47n:  RC=221µs, φ = −54.2° ✗
   ────────────────────────────────────────────────────────────── */
{ "RC Phase Shifter", "phase(H) = -arctan(wRC)",
  "Low-pass RC: H = Z_C/(Z_R+Z_C) -> phase = -arctan(wRC)",
  "For -45 deg at 1 kHz you need wRC = 1 -> RC = 159 us, so aim for ~150 us.",
  TOPO_RC_PHASE, 1.0, 1000.0, 0,
  2, {{'A',"R",CT_R}, {'B',"C",CT_C}},
  8, { {CT_R,1000,"1k Ohm"}, {CT_R,1500,"1.5k Ohm"},
       {CT_R,2200,"2.2k Ohm"}, {CT_R,4700,"4.7k Ohm"},
       {CT_C,47e-9,"47 nF"},  {CT_C,68e-9,"68 nF"},
       {CT_C,100e-9,"100 nF"},{CT_C,150e-9,"150 nF"} },
  {GOAL_PHASE_EQ, -45.0, 5.0, "phase = -45 deg  (+-5)"}
},

}; /* end of LEVELS[] */

/* ════════════════════════════════════════════════════════════════
 * SIMULATION ENGINE
 * ════════════════════════════════════════════════════════════════ */
typedef struct {
    int    valid;
    double vout;
    double iout_mA;
    double pout_mW;
    double mag;
    double phase_deg;
    double f0;
    double bw;
    int    goal_met;
    char   meas[120];
} SimResult;

/* Component in slot i, or NULL if the slot is empty */
static const Comp *slot_comp_ptr(const LevelDef *lv, const int sc[], int i) {
    if (i < 0 || i >= lv->n_slots) return NULL;
    if (sc[i] < 0 || sc[i] >= lv->n_tray) return NULL;
    return &lv->tray[sc[i]];
}

static SimResult simulate(const LevelDef *lv, const int sc[]) {
    SimResult r;
    memset(&r, 0, sizeof r);
    double omega = 2.0 * M_PI * lv->freq;

    /* Are all slots filled? */
    for (int i = 0; i < lv->n_slots; i++) {
        if (sc[i] < 0) {
            snprintf(r.meas, sizeof r.meas,
                "Fill all slots! (%c is empty)", 'A' + i);
            return r;
        }
    }
    r.valid = 1;

    /* Every slot must hold the right kind of part (e.g. L in the L slot) */
    for (int i = 0; i < lv->n_slots; i++) {
        const SlotDef *sd = &lv->slot_defs[i];
        const Comp *c = slot_comp_ptr(lv, sc, i);
        if (sd->need != CT_NONE && c->type != sd->need) {
            snprintf(r.meas, sizeof r.meas,
                "Slot %c (%s) must be a %s!",
                sd->name, sd->role, type_name(sd->need));
            return r;   /* valid = 1, goal_met = 0 */
        }
    }

    /* Impedance of every slot */
    Cx Z[MAX_SLOTS];
    for (int i = 0; i < lv->n_slots; i++) {
        const Comp *c = slot_comp_ptr(lv, sc, i);
        Z[i] = impedance(c->type, c->val, omega);
    }

    switch (lv->topo) {

    /* ─── Series voltage divider ──────────────────────────────
       Vout = Vs * Z_B / (Z_A + Z_B)   [complex]
       At DC the imaginary part is zero → real part = voltage
       ────────────────────────────────────────────────────────── */
    case TOPO_SERIES_DIV: {
        Cx Ztot = cx_add(Z[0], Z[1]);
        Cx Vout = cx_mul((Cx){lv->Vs, 0}, cx_div(Z[1], Ztot));
        r.vout      = Vout.re;
        r.mag       = cx_mag(Vout) / lv->Vs;
        r.phase_deg = cx_ang(Vout);
        snprintf(r.meas, sizeof r.meas,
            "Vout = %.3f V   R_A+R_B = %.0f Ohm",
            r.vout, Z[0].re + Z[1].re);
        break;
    }

    /* ─── Load effect ─────────────────────────────────────────
       Z_B_eff = Z_B || R_L
       Vout = Vs * Z_B_eff / (Z_A + Z_B_eff)
       ────────────────────────────────────────────────────────── */
    case TOPO_LOAD_EFFECT: {
        Cx Zload = (Cx){lv->fixed_val, 0};
        Cx ZBeff = cx_par(Z[1], Zload);
        Cx Ztot  = cx_add(Z[0], ZBeff);
        Cx Vout  = cx_mul((Cx){lv->Vs, 0}, cx_div(ZBeff, Ztot));
        r.vout     = Vout.re;
        r.iout_mA  = r.vout / lv->fixed_val * 1000.0;
        snprintf(r.meas, sizeof r.meas,
            "Vout = %.3f V   I_load = %.1f mA   Z_B_eff = %.1f Ohm",
            r.vout, r.iout_mA, cx_mag(ZBeff));
        break;
    }

    /* ─── Current divider ─────────────────────────────────────
       Vs is applied directly across each branch (ideal source)
       I_A = Vs / R_A,   I_B = Vs / R_B
       ────────────────────────────────────────────────────────── */
    case TOPO_CURRENT_DIV: {
        double Ra = slot_comp_ptr(lv, sc, 0)->val;
        double Rb = slot_comp_ptr(lv, sc, 1)->val;
        double Ia = lv->Vs / Ra * 1000.0;
        double Ib = lv->Vs / Rb * 1000.0;
        r.iout_mA = Ia;
        r.vout    = lv->Vs;
        snprintf(r.meas, sizeof r.meas,
            "I_A = %.1f mA   I_B = %.1f mA   I_total = %.1f mA",
            Ia, Ib, Ia + Ib);
        break;
    }

    /* ─── Maximum power transfer ──────────────────────────────
       I = Vs / (Rth + R_load)
       P = I² · R_load
       P_max = Vs² / (4·Rth)   [when R_load = Rth]
       ────────────────────────────────────────────────────────── */
    case TOPO_MAX_POWER: {
        double Rth = lv->fixed_val;
        double Rl  = slot_comp_ptr(lv, sc, 0)->val;
        double I   = lv->Vs / (Rth + Rl);
        r.pout_mW  = I * I * Rl * 1000.0;
        double Pmax= lv->Vs * lv->Vs / (4.0 * Rth) * 1000.0;
        snprintf(r.meas, sizeof r.meas,
            "P_load = %.1f mW   P_max = %.1f mW   efficiency = %.0f%% of max",
            r.pout_mW, Pmax, 100.0 * r.pout_mW / Pmax);
        break;
    }

    /* ─── LED driver ──────────────────────────────────────────
       I = (Vs − Vf) / R_series
       ────────────────────────────────────────────────────────── */
    case TOPO_LED_PROTECT: {
        double Vf = lv->fixed_val;
        double R  = slot_comp_ptr(lv, sc, 0)->val;
        r.iout_mA = (lv->Vs - Vf) / R * 1000.0;
        double Vr = r.iout_mA / 1000.0 * R;
        snprintf(r.meas, sizeof r.meas,
            "I_LED = %.2f mA   V_R = %.2f V   V_LED = %.1f V",
            r.iout_mA, Vr, Vf);
        break;
    }

    /* ─── RC low-pass ─────────────────────────────────────────
       Slot 0 = R (series), slot 1 = C (shunt)
       H(jω) = Z_C / (Z_R + Z_C) = 1 / (1 + jωRC)
       |H|   = 1 / √(1 + (ωRC)²)
       ∠H    = −arctan(ωRC)
       fc    = 1 / (2π·R·C)
       ────────────────────────────────────────────────────────── */
    case TOPO_RC_LOWPASS: {
        Cx Ztot = cx_add(Z[0], Z[1]);
        Cx H    = cx_div(Z[1], Ztot);
        r.mag       = cx_mag(H);
        r.phase_deg = cx_ang(H);
        double R = slot_comp_ptr(lv, sc, 0)->val;
        double C = slot_comp_ptr(lv, sc, 1)->val;
        double fc = 1.0 / (2.0 * M_PI * R * C);
        snprintf(r.meas, sizeof r.meas,
            "|H| = %.3f   phase = %.1f deg   fc = %.0f Hz",
            r.mag, r.phase_deg, fc);
        break;
    }

    /* ─── RC high-pass ────────────────────────────────────────
       Slot 0 = C (series), slot 1 = R (shunt)
       H(jω) = Z_R / (Z_C + Z_R) = jωRC / (1 + jωRC)
       |H|   = ωRC / √(1 + (ωRC)²)
       fc    = 1 / (2π·R·C)
       ────────────────────────────────────────────────────────── */
    case TOPO_RC_HIGHPASS: {
        Cx Ztot = cx_add(Z[0], Z[1]);
        Cx H    = cx_div(Z[1], Ztot);
        r.mag       = cx_mag(H);
        r.phase_deg = cx_ang(H);
        double C = slot_comp_ptr(lv, sc, 0)->val;
        double R = slot_comp_ptr(lv, sc, 1)->val;
        double fc = 1.0 / (2.0 * M_PI * R * C);
        snprintf(r.meas, sizeof r.meas,
            "|H| = %.3f   phase = %.1f deg   fc = %.0f Hz",
            r.mag, r.phase_deg, fc);
        break;
    }

    /* ─── Series RLC resonance ────────────────────────────────
       Slot 0 = L,  slot 1 = C,  R = fixed_val
       f0 = 1 / (2π√(LC))
       Q  = ω0·L / R
       ────────────────────────────────────────────────────────── */
    case TOPO_RLC_RESONANCE: {
        double L  = slot_comp_ptr(lv, sc, 0)->val;
        double C  = slot_comp_ptr(lv, sc, 1)->val;
        double R  = lv->fixed_val;
        r.f0      = 1.0 / (2.0 * M_PI * sqrt(L * C));
        double Q  = (2.0 * M_PI * r.f0 * L) / R;
        r.bw      = r.f0 / Q;
        snprintf(r.meas, sizeof r.meas,
            "f0 = %.2f kHz   Q = %.2f   BW = %.2f kHz",
            r.f0 / 1000.0, Q, r.bw / 1000.0);
        break;
    }

    /* ─── RLC bandwidth ───────────────────────────────────────
       Slot 0 = R.   L = 1 mH,  C = 1 µF (fixed)
       f0 = 1 / (2π√(LC)) ≈ 5.03 kHz
       BW = R / (2π·L)   [series RLC]
       Q  = f0 / BW
       ────────────────────────────────────────────────────────── */
    case TOPO_RLC_BANDPASS: {
        double R   = slot_comp_ptr(lv, sc, 0)->val;
        double L   = 1e-3;   /* fixed 1 mH */
        double C   = 1e-6;   /* fixed 1 µF */
        r.f0       = 1.0 / (2.0 * M_PI * sqrt(L * C));
        r.bw       = R / (2.0 * M_PI * L);
        double Q   = r.f0 / r.bw;
        snprintf(r.meas, sizeof r.meas,
            "f0 = %.2f kHz   BW = %.2f kHz   Q = %.2f",
            r.f0 / 1000.0, r.bw / 1000.0, Q);
        break;
    }

    /* ─── RC phase shifter ────────────────────────────────────
       RC low-pass: H = Z_C / (Z_R + Z_C)
       ∠H = −arctan(ωRC)   →   −45° when ωRC = 1
       ────────────────────────────────────────────────────────── */
    case TOPO_RC_PHASE: {
        Cx Ztot = cx_add(Z[0], Z[1]);
        Cx H    = cx_div(Z[1], Ztot);
        r.mag       = cx_mag(H);
        r.phase_deg = cx_ang(H);
        double R = slot_comp_ptr(lv, sc, 0)->val;
        double C = slot_comp_ptr(lv, sc, 1)->val;
        double RC = R * C;
        snprintf(r.meas, sizeof r.meas,
            "phase = %.1f deg   |H| = %.3f   RC = %.1f us   wRC = %.3f",
            r.phase_deg, r.mag, RC * 1e6, omega * RC);
        break;
    }

    } /* switch(topo) */

    /* ── Goal check ── */
    Goal g = lv->goal;
    switch (g.type) {
        case GOAL_VOUT_EQ:       r.goal_met = fabs(r.vout - g.p1) <= g.p2;        break;
        case GOAL_VOUT_GE:       r.goal_met = r.vout >= g.p1;                     break;
        case GOAL_CURRENT_RANGE: r.goal_met = r.iout_mA>=g.p1 && r.iout_mA<=g.p2; break;
        case GOAL_POWER_MAX:     r.goal_met = r.pout_mW >= g.p1;                  break;
        case GOAL_MAG_LE:        r.goal_met = r.mag <= g.p1;                      break;
        case GOAL_F0_RANGE:      r.goal_met = r.f0>=g.p1 && r.f0<=g.p2;           break;
        case GOAL_BW_LE:         r.goal_met = r.bw <= g.p1;                       break;
        case GOAL_PHASE_EQ:      r.goal_met = fabs(r.phase_deg-g.p1) <= g.p2;     break;
    }

    return r;
}

/* ════════════════════════════════════════════════════════════════
 * GAME STATE
 * ════════════════════════════════════════════════════════════════ */
static int  g_lvl      = 0;
static int  g_score    = 0;
static int  g_won      = 0;
static int  g_sc[MAX_SLOTS];   /* tray index per slot, -1 = empty */
static int  g_cur_slot = 0;    /* currently selected slot         */
static time_t g_t0;
static int  g_total_t  = 0;

static char g_msg[256];
static int  g_msg_t    = 0;
static int  g_msg_ok   = 0;

static void reset_level(int lv) {
    g_lvl  = lv;
    g_won  = 0;
    g_t0   = time(NULL);
    g_cur_slot = 0;
    for (int i = 0; i < MAX_SLOTS; i++) g_sc[i] = -1;
    const LevelDef *L = &LEVELS[lv];
    snprintf(g_msg, sizeof g_msg, "%s", L->hint);
    g_msg_t = 400; g_msg_ok = 0;
}

/* ════════════════════════════════════════════════════════════════
 * DRAWING
 * Layout: Title → Level → CIRCUIT → GOAL & MEASUREMENT → TRAY → CONTROLS
 * ════════════════════════════════════════════════════════════════ */

/* ── Horizontal section header ── */
static void hline_section(int y, const char *title) {
    int H, W;
    getmaxyx(stdscr, H, W);
    (void)H;
    attron(COLOR_PAIR(CP_BAR)|A_BOLD);
    int tlen = (int)strlen(title);
    mvprintw(y, 0, "%s", title);
    for (int x = tlen; x < W; x++) mvaddch(y, x, '-');
    attroff(COLOR_PAIR(CP_BAR)|A_BOLD);
}

/* ── Part color on/off ── */
static void cc(CompType t, int on) {
    int cp = CP_SLOT;
    switch(t) {
        case CT_R:   cp = CP_R;   break;
        case CT_C:   cp = CP_C;   break;
        case CT_L:   cp = CP_L;   break;
        case CT_LED: cp = CP_LED; break;
        default:     cp = CP_SLOT; break;
    }
    if (on) attron(COLOR_PAIR(cp)|A_BOLD);
    else    attroff(COLOR_PAIR(cp)|A_BOLD);
}

/* ── Slot box: "[A:1k Ohm   ]", inverted colors when selected ── */
static void print_slot(const LevelDef *lv, int si) {
    int ci  = g_sc[si];
    int sel = (si == g_cur_slot);
    CompType t = CT_NONE;
    char lbl[20];
    if (ci >= 0 && ci < lv->n_tray) {
        t = lv->tray[ci].type;
        snprintf(lbl, sizeof lbl, "%-10s", lv->tray[ci].label);
    } else {
        snprintf(lbl, sizeof lbl, "   ???    ");
    }
    if (sel) { attron(COLOR_PAIR(CP_SEL)|A_BOLD); }
    else     { cc(t, 1); }
    printw("[%c:%-10s]", lv->slot_defs[si].name, lbl);
    if (sel) { attroff(COLOR_PAIR(CP_SEL)|A_BOLD); }
    else     { cc(t, 0); }
}

/* ── Center column ── */
static int midx(void) {
    int H, W;
    getmaxyx(stdscr, H, W);
    (void)H;
    return W / 2;
}

/* ════════════════════════════════════════════════════════════════
 * ASCII SCHEMATICS
 *
 * Each topology has its own schematic.
 * The returned y is the next free row.
 * ════════════════════════════════════════════════════════════════ */

/* ── Series voltage divider ──────────────────────────────────────
 *        Vs
 *         |
 *    [A: R_top ]
 *         |
 *         +------- Vout = X V    (node between R_top and R_bot)
 *         |
 *    [B: R_bot ]
 *         |
 *        GND
 * ─────────────────────────────────────────────────────────────── */
static int sch_series_div(int y, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    mvprintw(y, cx, "|");                                        y++;
    move(y, cx-6); print_slot(lv, 0);                           y++;
    mvprintw(y, cx, "|");                                        y++;
    mvprintw(y, cx, "+");
    attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
    if (sr && sr->valid)
        mvprintw(y, cx+1, "------- Vout = %.3f V", sr->vout);
    else
        mvprintw(y, cx+1, "------- Vout = ?");
    attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);                         y++;
    mvprintw(y, cx, "|");                                        y++;
    move(y, cx-6); print_slot(lv, 1);                           y++;
    mvprintw(y, cx, "|");                                        y++;
    attron(A_BOLD); mvprintw(y, cx-1, "GND"); attroff(A_BOLD);  y++;
    return y;
}

/* ── Load effect (parallel R_L) ──────────────────────────────────
 *       Vs
 *        |
 *   [A: R_series]         <- series resistor
 *        |
 *        +-------- Vout   <- top node; R_B and R_L both start here
 *        |         |
 *  [B: R_B  ]  [RL=XXOhm fixed]
 *        |         |
 *        +----+----+      <- bottom node: both legs join here
 *             |
 *            GND          <- common reference
 * ─────────────────────────────────────────────────────────────── */
static int sch_load_effect(int y, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    int L = cx - 9;    /* left leg x  */
    int R = cx + 9;    /* right leg x */

    /* series resistor feeding the top bar */
    mvprintw(y, cx, "|");                                         y++;
    move(y, cx-6); print_slot(lv, 0);                            y++;
    mvprintw(y, cx, "|");                                         y++;

    /* TOP BAR: L ---+--- R ------- Vout
       Same style as the bottom bar; Vout leaves on the right */
    mvprintw(y, L, "+");
    for (int x = L+1; x < cx; x++) mvaddch(y, x, '-');
    mvprintw(y, cx, "+");
    for (int x = cx+1; x < R; x++) mvaddch(y, x, '-');
    mvprintw(y, R, "+");
    attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
    if (sr && sr->valid)
        mvprintw(y, R+1, "------ Vout = %.3f V", sr->vout);
    else
        mvprintw(y, R+1, "------ Vout = ?");
    attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);                          y++;

    /* upper wires of both legs */
    mvprintw(y, L, "|"); mvprintw(y, R, "|");                     y++;

    /* parallel parts: left = [B], right = R_L */
    move(y, L-6); print_slot(lv, 1);
    attron(A_DIM);
    mvprintw(y, R-5, "[RL=%.0fOhm]", lv->fixed_val);
    attroff(A_DIM);                                               y++;

    /* lower wires of both legs */
    mvprintw(y, L, "|"); mvprintw(y, R, "|");                     y++;

    /* BOTTOM BAR: L ---+--- R */
    mvprintw(y, L, "+");
    for (int x = L+1; x < cx; x++) mvaddch(y, x, '-');
    mvprintw(y, cx, "+");
    for (int x = cx+1; x < R; x++) mvaddch(y, x, '-');
    mvprintw(y, R, "+");                                           y++;

    /* GND from the center */
    mvprintw(y, cx, "|");                                          y++;
    attron(A_BOLD); mvprintw(y, cx-1, "GND"); attroff(A_BOLD);    y++;
    return y;
}

/* ── RC / CR filters (low-pass, high-pass, phase) ───────────────
 *     Slot 0 = series arm,  slot 1 = shunt arm (to GND)
 *
 *        Vin~
 *          |
 *     [A: series part]
 *          |
 *          +------- Vout = |H|∠φ    (top of the shunt part)
 *          |
 *     [B: shunt part]               (bottom end goes to GND)
 *          |
 *         GND
 * ─────────────────────────────────────────────────────────────── */
static int sch_rc_filter(int y, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    mvprintw(y, cx, "|");                                        y++;
    move(y, cx-6); print_slot(lv, 0);                           y++;
    mvprintw(y, cx, "|");                                        y++;
    /* node + |H| readout to the right */
    mvprintw(y, cx, "+");
    attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
    if (sr && sr->valid)
        mvprintw(y, cx+1, "--- |H|=%.3f  phase=%.1f deg",
                 sr->mag, sr->phase_deg);
    else
        mvprintw(y, cx+1, "--- Vout = ?");
    attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);                         y++;
    mvprintw(y, cx, "|");                                        y++;
    move(y, cx-6); print_slot(lv, 1);                           y++;
    mvprintw(y, cx, "|");                                        y++;
    attron(A_BOLD); mvprintw(y, cx-1, "GND"); attroff(A_BOLD);  y++;
    return y;
}

/* ── Current divider ─────────────────────────────────────────────
 *        Vs
 *         |
 *    ─────+─────
 *    |         |
 * [A:Ra]    [B:Rb]
 *  Ia mA     Ib mA
 *    |         |
 *    +─────────+
 *         |
 *        GND
 * ─────────────────────────────────────────────────────────────── */
static int sch_current_div(int y, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    int L  = cx - 9;    /* left branch wire  */
    int R  = cx + 9;    /* right branch wire */

    /* wire from Vs down to the top bar */
    mvprintw(y, cx, "|");                                         y++;

    /* TOP BAR: L ---+--- R */
    mvprintw(y, L, "+");
    for (int x = L+1; x < cx; x++) mvaddch(y, x, '-');
    mvprintw(y, cx, "+");
    for (int x = cx+1; x < R; x++) mvaddch(y, x, '-');
    mvprintw(y, R, "+");                                          y++;

    /* upper wires of both branches */
    mvprintw(y, L, "|"); mvprintw(y, R, "|");                    y++;

    /* slots: A on the left, B on the right */
    move(y, L-6); print_slot(lv, 0);
    move(y, R-6); print_slot(lv, 1);                             y++;

    /* branch currents */
    if (sr && sr->valid && g_sc[1] >= 0) {
        double Ib = lv->Vs / lv->tray[g_sc[1]].val * 1000.0;
        attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
        mvprintw(y, L-4, "%.1fmA", sr->iout_mA);
        mvprintw(y, R-4, "%.1fmA", Ib);
        attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);
    }                                                             y++;

    /* lower wires of both branches */
    mvprintw(y, L, "|"); mvprintw(y, R, "|");                    y++;

    /* BOTTOM BAR: L ---+--- R  (same as the top bar) */
    mvprintw(y, L, "+");
    for (int x = L+1; x < cx; x++) mvaddch(y, x, '-');
    mvprintw(y, cx, "+");
    for (int x = cx+1; x < R; x++) mvaddch(y, x, '-');
    mvprintw(y, R, "+");                                          y++;

    /* GND from the center */
    mvprintw(y, cx, "|");                                         y++;
    attron(A_BOLD); mvprintw(y, cx-1, "GND"); attroff(A_BOLD);   y++;
    return y;
}

/* ── Maximum power transfer ──────────────────────────────────────
 *    Thevenin equivalent circuit:
 *
 *        Vs = 10V
 *         |
 *    [Rth = 50 Ohm  fixed]   <- source-side internal resistance
 *         |
 *    [A: R_load  ]           <- load; P_load is measured here
 *         |
 *        GND
 *
 *    P_max = Vs²/(4·Rth)  →  when R_load = Rth
 * ─────────────────────────────────────────────────────────────── */
static int sch_max_power(int y, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    mvprintw(y, cx, "|");                                          y++;
    attron(A_DIM);
    mvprintw(y, cx-11, "[Rth = %.0f Ohm  (fixed)]", lv->fixed_val);
    attroff(A_DIM);                                                y++;
    mvprintw(y, cx, "|");                                          y++;
    move(y, cx-6); print_slot(lv, 0);                             y++;
    mvprintw(y, cx, "|");                                          y++;
    attron(A_BOLD); mvprintw(y, cx-1, "GND"); attroff(A_BOLD);    y++;
    y++;
    if (sr && sr->valid) {
        attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
        mvprintw(y, 2, "%s", sr->meas);
        attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);                       y++;
    }
    y++;
    return y;
}

/* ── LED driver ──────────────────────────────────────────────────
 *        Vs
 *         |
 *    [A: R_series]    I = (Vs - Vf) / R
 *         |
 *    [LED  Vf=2V]
 *         |
 *        GND
 * ─────────────────────────────────────────────────────────────── */
static int sch_led(int y, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    mvprintw(y, cx, "|");                                         y++;
    move(y, cx-6); print_slot(lv, 0);                            y++;
    mvprintw(y, cx, "|");                                         y++;
    attron(COLOR_PAIR(CP_LED)|A_BOLD);
    mvprintw(y, cx-9, "[ LED  Vf = %.1f V ]", lv->fixed_val);
    attroff(COLOR_PAIR(CP_LED)|A_BOLD);                           y++;
    mvprintw(y, cx, "|");                                         y++;
    attron(A_BOLD); mvprintw(y, cx-1, "GND"); attroff(A_BOLD);   y++;
    if (sr && sr->valid) {
        y++;
        attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
        mvprintw(y, 2, "%s", sr->meas);
        attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);                      y++;
    }
    return y;
}

/* ── Series RLC (resonance or bandwidth) ─────────────────────────
 *
 *  Vin~ ──[R=100 Ohm fixed]──[A: L]──[B: C]──GND
 *
 *           f0 = 1/(2π√LC)   Q = ω0·L/R
 * ─────────────────────────────────────────────────────────────── */
static int sch_rlc_series(int y, const LevelDef *lv, const SimResult *sr,
                          int bw_mode) {
    /* bw_mode = 1 → L and C are shown as fixed, only the R slot exists */
    y++;
    int x = 2;
    attron(COLOR_PAIR(CP_WIRE)|A_BOLD);
    mvprintw(y, x, "Vin~"); x += 4;
    attroff(COLOR_PAIR(CP_WIRE)|A_BOLD);
    mvprintw(y, x, "--"); x += 2;
    if (bw_mode) {
        move(y, x); print_slot(lv, 0); x += 14;
        attron(A_DIM);
        mvprintw(y, x, "--[L=1mH]--[C=1uF]");
        attroff(A_DIM);
        x += 19;
    } else {
        attron(A_DIM);
        {
            char rbuf[24];
            snprintf(rbuf, sizeof rbuf, "[R=%.0fOhm fixed]", lv->fixed_val);
            mvprintw(y, x, "%s", rbuf);
            x += (int)strlen(rbuf);
        }
        attroff(A_DIM);
        mvprintw(y, x, "--"); x += 2;
        move(y, x); print_slot(lv, 0); x += 14;
        mvprintw(y, x, "--"); x += 2;
        move(y, x); print_slot(lv, 1); x += 14;
    }
    mvprintw(y, x, "--+--"); x += 5;
    attron(A_BOLD); mvprintw(y, x, "GND"); attroff(A_BOLD);
    y++;
    y++;
    if (sr && sr->valid) {
        attron(COLOR_PAIR(CP_MEAS)|A_BOLD);
        mvprintw(y, 2, "%s", sr->meas);
        attroff(COLOR_PAIR(CP_MEAS)|A_BOLD);                      y++;
    }
    y++;
    return y;
}

/* ── Main schematic dispatcher ── */
static int draw_schematic(int y0, const LevelDef *lv, const SimResult *sr) {
    int cx = midx();
    int y = y0 + 1;   /* +1: top padding */

    /* Source label + AC/DC badge */
    attron(COLOR_PAIR(CP_WIRE)|A_BOLD);
    if (lv->freq > 0)
        mvprintw(y, cx-14, "[AC~]  Vin = %.0f V    f = %.0f Hz", lv->Vs, lv->freq);
    else
        mvprintw(y, cx-10, "[DC]   Vs  = %.0f V", lv->Vs);
    attroff(COLOR_PAIR(CP_WIRE)|A_BOLD);
    y++;

    switch (lv->topo) {
        case TOPO_SERIES_DIV:   y = sch_series_div(y, lv, sr);       break;
        case TOPO_LOAD_EFFECT:  y = sch_load_effect(y, lv, sr);      break;
        case TOPO_CURRENT_DIV:  y = sch_current_div(y, lv, sr);      break;
        case TOPO_MAX_POWER:    y = sch_max_power(y, lv, sr);        break;
        case TOPO_LED_PROTECT:  y = sch_led(y, lv, sr);              break;
        case TOPO_RC_LOWPASS:
        case TOPO_RC_HIGHPASS:
        case TOPO_RC_PHASE:     y = sch_rc_filter(y, lv, sr);        break;
        case TOPO_RLC_RESONANCE:y = sch_rlc_series(y, lv, sr, 0);    break;
        case TOPO_RLC_BANDPASS: y = sch_rlc_series(y, lv, sr, 1);    break;
    }
    return y + 1;   /* +1: bottom padding */
}

/* ── Tray section ── */
static int draw_tray(int y0, const LevelDef *lv) {
    int y = y0;
    for (int i = 0; i < lv->n_tray; i++) {
        const Comp *c = &lv->tray[i];
        int asgn = -1;
        for (int s = 0; s < lv->n_slots; s++)
            if (g_sc[s] == i) asgn = s;

        attron(A_BOLD);
        mvprintw(y, 2, "%2d)", i + 1);
        attroff(A_BOLD);

        if (asgn >= 0) attron(A_DIM);
        cc(c->type, 1);
        mvprintw(y, 6, "%-13s", c->label);
        cc(c->type, 0);
        if (asgn >= 0) attroff(A_DIM);

        if (asgn >= 0) {
            attron(COLOR_PAIR(CP_OK)|A_BOLD);
            mvprintw(y, 20, "-> slot [%c]", 'A' + asgn);
            attroff(COLOR_PAIR(CP_OK)|A_BOLD);
        }
        y++;
    }
    return y;
}

/* ════════════════════════════════════════════════════════════════
 * MAIN DRAW FUNCTION
 * ════════════════════════════════════════════════════════════════ */
static void draw(void) {
    const LevelDef *lv = &LEVELS[g_lvl];
    SimResult sr = simulate(lv, g_sc);
    int elapsed = (int)(time(NULL) - g_t0);
    int W, H;
    getmaxyx(stdscr, H, W);
    (void)H;

    erase();

    /* ── Title bar ── */
    attron(COLOR_PAIR(CP_BAR)|A_BOLD);
    mvprintw(0, 0, " VOLTS");
    char hdr[64];
    snprintf(hdr, sizeof hdr, " Level:%d/%d  Score:%-5d  %02d:%02d ",
             g_lvl+1, N_LEVELS, g_score, elapsed/60, elapsed%60);
    mvprintw(0, W - (int)strlen(hdr), "%s", hdr);
    attroff(COLOR_PAIR(CP_BAR)|A_BOLD);

    /* ── Level title ── */
    attron(A_BOLD);
    mvprintw(1, 2, "Level %d: %s", g_lvl + 1, lv->title);
    attroff(A_BOLD);
    attron(A_DIM);
    mvprintw(2, 2, "Concept: %s", lv->concept);
    attroff(A_DIM);

    /* ── CIRCUIT section ── */
    hline_section(3, " CIRCUIT ");
    int y = draw_schematic(4, lv, &sr);

    /* ── GOAL & MEASUREMENT section ── */
    hline_section(y, " GOAL & MEASUREMENT "); y++;
    y++;  /* top padding */

    attron(A_BOLD);
    mvprintw(y, 2, "Goal   : %s", lv->goal.desc);   y++;
    attroff(A_BOLD);

    if (sr.valid) {
        int cp = sr.goal_met ? CP_OK : CP_ERR;
        attron(COLOR_PAIR(cp)|A_BOLD);
        mvprintw(y, 2, "Measure: %s", sr.meas);     y++;
        attroff(COLOR_PAIR(cp)|A_BOLD);
    } else {
        attron(A_DIM);
        mvprintw(y, 2, "Measure: %s", sr.meas);     y++;
        attroff(A_DIM);
    }

    y++;  /* spacing */
    attron(A_DIM);
    mvprintw(y, 2, "Theory : %s", lv->theory);      y++;
    mvprintw(y, 2, "Hint   : %s", lv->hint);        y++;
    attroff(A_DIM);
    y++;  /* bottom padding */

    /* ── TRAY section ── */
    hline_section(y, " TRAY  (1-9 pick part, A-D select slot) "); y++;
    y++;
    y = draw_tray(y, lv);
    y++;

    /* ── CONTROLS ── */
    hline_section(y, " CONTROLS "); y++;
    y++;
    attron(A_BOLD);
    mvprintw(y, 2, "A-D: slot   1-9: assign part   X: clear   Enter: check   N: next   Q: quit");
    attroff(A_BOLD);
    y += 2;

    /* ── Status message ── */
    if (g_msg_t > 0) {
        int cp = g_msg_ok ? CP_OK : CP_ERR;
        attron(COLOR_PAIR(cp)|A_BOLD);
        mvprintw(y, 0, " >> %s", g_msg);
        attroff(COLOR_PAIR(cp)|A_BOLD);
        g_msg_t--;
    }

    refresh();
}

/* ════════════════════════════════════════════════════════════════
 * INPUT HANDLING
 * ════════════════════════════════════════════════════════════════ */
static void handle_key(int ch) {
    const LevelDef *lv = &LEVELS[g_lvl];

    if (g_won) {
        if (ch == 'n' || ch == 'N') {
            if (g_lvl < N_LEVELS - 1) { reset_level(g_lvl + 1); }
            else {
                snprintf(g_msg, sizeof g_msg,
                    "ALL LEVELS COMPLETE! Final score: %d", g_score);
                g_msg_t = 800; g_msg_ok = 1;
            }
        }
        return;
    }

    /* Slot selection: A / B / C / D */
    if (ch >= 'a' && ch <= 'd') ch -= 32;
    if (ch >= 'A' && ch <= 'D') {
        int s = ch - 'A';
        if (s < lv->n_slots) g_cur_slot = s;
        return;
    }

    /* Part assignment: 1-9 (and 0 = 10th part) */
    if (ch >= '1' && ch <= '9') {
        int idx = ch - '1';
        if (idx < lv->n_tray) g_sc[g_cur_slot] = idx;
        return;
    }
    if (ch == '0') {
        if (9 < lv->n_tray) g_sc[g_cur_slot] = 9;
        return;
    }

    /* Clear */
    if (ch == 'x' || ch == 'X' || ch == KEY_DC || ch == KEY_BACKSPACE)
        g_sc[g_cur_slot] = -1;

    /* Check / submit */
    if (ch == '\n' || ch == KEY_ENTER) {
        SimResult sr = simulate(lv, g_sc);
        if (!sr.valid) {
            snprintf(g_msg, sizeof g_msg, "%s", sr.meas);
            g_msg_t = 100; g_msg_ok = 0;
            return;
        }
        if (sr.goal_met) {
            int t   = (int)(time(NULL) - g_t0);
            g_total_t += t;
            int pts = 500 + (9 - g_lvl) * 50 - t * 3;
            if (pts < 50) pts = 50;
            g_score += pts;
            g_won    = 1;
            snprintf(g_msg, sizeof g_msg,
                "CORRECT! +%d pts (%ds) %s [N]",
                pts, t, sr.meas);
            g_msg_t = 600; g_msg_ok = 1;
        } else {
            snprintf(g_msg, sizeof g_msg,
                "WRONG: %s   Goal: %s", sr.meas, lv->goal.desc);
            g_msg_t = 160; g_msg_ok = 0;
        }
    }
}

/* ════════════════════════════════════════════════════════════════
 * MAIN
 * ════════════════════════════════════════════════════════════════ */
int main(void) {
    initscr();
    cbreak(); noecho();
    keypad(stdscr, TRUE);
    curs_set(0);
    timeout(100);

    if (!has_colors()) {
        endwin();
        puts("This terminal does not support colors.");
        return 1;
    }
    start_color();
    use_default_colors();

    init_pair(CP_TITLE,  COLOR_WHITE,   -1);
    init_pair(CP_SLOT,   COLOR_WHITE,   -1);
    init_pair(CP_SEL,    COLOR_BLACK,   COLOR_WHITE);
    init_pair(CP_R,      COLOR_YELLOW,  -1);
    init_pair(CP_C,      COLOR_CYAN,    -1);
    init_pair(CP_L,      COLOR_MAGENTA, -1);
    init_pair(CP_LED,    COLOR_GREEN,   -1);
    init_pair(CP_OK,     COLOR_GREEN,   -1);
    init_pair(CP_ERR,    COLOR_RED,     -1);
    init_pair(CP_MEAS,   COLOR_CYAN,    -1);
    init_pair(CP_DIM,    COLOR_WHITE,   -1);
    init_pair(CP_WIRE,   COLOR_WHITE,   -1);
    init_pair(CP_BAR,    COLOR_BLACK,   COLOR_CYAN);
    init_pair(CP_TRAY,   COLOR_YELLOW,  -1);

    reset_level(0);

    int ch;
    while ((ch = getch()) != 'q' && ch != 'Q') {
        if (ch != ERR) handle_key(ch);
        draw();
    }

    endwin();
    printf("\n  Bye.  Final score: %d  |  Total time: %02d:%02d\n\n",
           g_score, g_total_t/60, g_total_t%60);
    return 0;
}
