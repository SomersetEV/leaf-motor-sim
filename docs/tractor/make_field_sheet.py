"""Printable A4 field sheet for tractor captures R1, R2 and R4 (plan section 9, M7).

    python make_field_sheet.py [OUT.pdf]

Writes M7_R1_R2_R4_field_sheet.pdf next to this script by default. Needs
reportlab, which is not in tools/requirements.txt (pip install reportlab).
"""
import sys
from pathlib import Path

from fieldsheet import *  # noqa: F401,F403

OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent / "M7_R1_R2_R4_field_sheet.pdf")


s = []
s.append(p("Tractor captures R1, R2 and R4", H1))
s.append(p("Date: ____________________ &nbsp;&nbsp;&nbsp; Ambient temp: ________ &nbsp;&nbsp;&nbsp; "
           "Battery SoC: ________ %", BODY))
s.append(Spacer(1, 4))
s.append(p("All three runs are done parked, in neutral with the PTO out, so only the motor and the "
           "gearbox input spin. About 15 minutes in all. Write down the clock time at the start of every "
           "run: one session file may hold several runs, and the time is how they are found afterwards.",
           SMALL))

s.append(p("Before you start", H2))
s.append(checklist([
    "Logger shown to capture without drops (bench check, then a minute parked in run mode on the tractor). "
    "Phone connected so the log has clock time.",
    "Zombie on the <b>gov-logging</b> firmware. If it was just flashed: set <b>govFlt = 4</b>, save, "
    "power-cycle (the ID 162 clash otherwise loads 2046 and the governor commands full regen).",
    "Tractor stationary: <b>GovLog</b> on, then check the MG charger's DC-DC output stays up.",
    "Handbrake on, wheels chocked, gearbox in <b>neutral</b>, PTO <b>out</b>, nobody near the PTO shaft.",
    "Gearbox warm (friction depends on oil temperature). Warm-up drive done? &nbsp; yes / no",
    "Write down the settings you will change, so they can be put back:<br/>"
    "revlim ________ &nbsp;&nbsp; off-throttle regen setting (name) ______________ = ________",
]))

s.append(p("R1 &nbsp;No-load acceleration", H2))
s.append(p("Gives the inertia J of the motor and gearbox input, and friction at three speeds. "
           "13 Nov 2026 gave J 0.055 kg m2, Tc 4.8 Nm, b 0.045 Nm s/rad: repeat only with a logger "
           "proven to keep every frame.", GIVES))
s.append(steps([
    "Set <b>revlim = 3000</b> so the governor stays out of the way.",
    "From standstill, press the throttle to about <b>3 %</b> (potnom) and hold it still. The shaft is light "
    "and friction is nearly flat, so it speeds up and <b>settles at its own speed</b> within about 5 s, "
    "somewhere between 1500 and 2500 rpm. Do not chase an exact speed: 0.1 % of pedal moves it about 110 rpm. "
    "Hold the settled speed for about <b>5 s</b>.",
    "Release the throttle fully and let it coast to a stop. That coast is R2 (next section), so let it run "
    "down without touching anything. It takes about a second.",
    "<b>Abort:</b> if speed passes <b>2600 rpm</b>, release at once.",
    "Repeat <b>three times</b> at about <b>2.5, 3 and 3.5 %</b>. Note the pedal % and the settled rpm.",
]))
s.append(Spacer(1, 4))
s.append(record("R1", ["R1 / R2 #1", "R1 / R2 #2", "R1 / R2 #3"], rpm_col="Settled rpm"))

s.append(p("R2 &nbsp;Coast-down", H2))
s.append(p("Gives the friction terms Tc, b and c.", GIVES))
s.append(steps([
    "If practical, set the Zombie's <b>off-throttle regen to zero</b> for R1/R2. Otherwise regen torque is "
    "mixed in with the friction. If you leave it on, write ON here: ______",
    "From about 2000 rpm (the end of each R1), throttle <b>fully released</b>, hands off, until the shaft "
    "stops. No brake, no clutch, no gear change.",
    "R2 is recorded in the same rows as R1 above. Note in the rows if anything disturbed a coast.",
]))

s.append(PageBreak())

s.append(p("R4 &nbsp;Fast hit on the limit", H2))
s.append(p("Reference for overshoot above revlim, ringing, and any dip-and-creep. The bench repeats it with "
           "the same gains (validation step 2).", GIVES))
s.append(boxed([
    p("<b>Set back to the working governor settings first:</b>", BODY),
    Spacer(1, 2),
    checklist([
        "<b>revlim = 2200</b>",
        "Off-throttle regen back to its usual value (written down on page 1)",
        "Gains as normal: govKp 11, govKi 0, govKd 0, govImax 100, regenmax -30, throtramp 10, govFlt 4. "
        "Different? Write here: ________________________________",
    ], width=W - 12 * mm),
    Spacer(1, 3),
    p("<b>Expect hunting.</b> In neutral the shaft is so light that these gains cannot settle it: on 13 Nov "
      "speed swung about 1600 to 2470 rpm with the pedal held. That is what this run records.", SMALL),
]))
s.append(Spacer(1, 4))
s.append(steps([
    "Still parked, neutral, PTO out. Start from <b>standstill</b>, throttle closed.",
    "Step the throttle quickly to about <b>6 %</b> (potnom) in one movement, and hold it there. "
    "<b>Never full throttle:</b> on 13 Nov, 18 % overshot to 3460 rpm against revlim 2200, and full "
    "throttle requests about five times that torque.",
    "Hold for about <b>5 s</b>, then release and let it stop.",
    "<b>Abort:</b> if speed passes <b>2700 rpm</b> (revlim + 500), release at once and note it.",
    "Repeat <b>three times</b> at the same pedal. Wait for the shaft to stop between repeats.",
]))
s.append(Spacer(1, 4))
s.append(record("R4", ["R4 #1", "R4 #2", "R4 #3"], extra_col="Pedal %", rpm_col="Peak rpm"))

s.append(p("Afterwards", H2))
s.append(checklist([
    "revlim back to <b>2200</b>, off-throttle regen back to normal. Saved.",
    "GovLog: leave on / turn off (circle one). DC-DC still fine?",
    "Copy the session files off the logger. Put each run's clock times, pedal % and notes with them "
    "(photo of this sheet is fine).",
    "On the PC: <font name='Courier'>python tools/loglib.py session_XXXX.csv</font> - the 0x1D4 and 0x4A1 "
    "gap counts should be near 0, and dropped 0x4A1 frames 0.",
]))

s.append(p("What happens to the logs", H2))
s.append(p("<font name='Courier'>extract_params.py</font> fits J from R1 and Tc, b, c from R2, and writes the "
           "simulator settings. The bench then repeats R1 open loop (validation step 1, within 5 %) and R4 with "
           "the gains and pedal recorded in the log (step 2: overshoot within 20 % or 30 rpm, whichever is "
           "larger, and similar dip-and-creep). Only once those match are bench sweeps trusted for tuning.", SMALL))

s.append(p("Notes", H2))
s.append(notes())

build(OUT, s, "Tractor captures R1, R2 and R4",
      "Leyland 255 governor work - tractor captures R1, R2, R4 (SIMULATOR_PLAN.md section 9, milestone M7)")
