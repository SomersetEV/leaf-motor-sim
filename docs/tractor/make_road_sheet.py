"""Printable A4 field sheet for the tractor road runs D1, D2 and D3 (plan section 9, M7).

    python make_road_sheet.py [OUT.pdf]

Writes M7_road_D1_D3_field_sheet.pdf next to this script by default. Needs
reportlab (pip install reportlab). The pedal steps and expectations come from
the harness and loopcalc at the gear inertias, 14 Nov 2026
(docs/tractor/road_runs_plan.md).
"""
import sys
from pathlib import Path

from fieldsheet import *  # noqa: F401,F403

OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent / "M7_road_D1_D3_field_sheet.pdf")

s = []
s.append(p("Tractor road runs D1, D2 and D3", H1))
s.append(p("Date: ______________ &nbsp;&nbsp; Ambient temp: ______ &nbsp;&nbsp; "
           "Battery SoC: ________ % &nbsp;&nbsp; Trailer: none / ________ kg", BODY))
s.append(Spacer(1, 4))
s.append(p("On the road, the shaft the governor drives is the whole tractor seen through the gearbox: 3 to 60 "
           "times heavier than the neutral shaft of 13 Nov, depending on gear. These runs test the loop model "
           "across that range. Allow about 40 minutes, on a quiet road. Traffic always comes first, and an "
           "abort rule overrides everything else. Write down the clock time at the start of every run.", SMALL))

s.append(p("Before you start", H2))
s.append(checklist([
    "Logger: note which firmware build it runs: ____________. Shown to keep every frame (bench, then a "
    "minute parked in run mode). Phone connected so the log has clock time.",
    "Tractor stationary: <b>GovLog</b> on, then check the MG charger's DC-DC output stays up.",
    "Settings as found: revlim ______ govKp ______ govKi ______ govKd ______ govFlt ______ "
    "regenmax ______ throtramp ______",
    "A quiet, straight, <b>flat</b> stretch of at least 500 m that you can drive both ways: ______________. "
    "A <b>steady hill</b> for D3: ______________ (rough gradient ____ %).",
]))
s.append(Spacer(1, 3))
s.append(p("<b>Pick three gears</b> by road speed at 1800 motor rpm (speedometer or phone GPS). That speed "
           "tells us the inertia of each gear.", BODY))
s.append(Spacer(1, 2))
s.append(table(["Use", "Aim for, at 1800 rpm", "Gear", "km/h at 1800 rpm"],
               [["A (road)", "15 km/h or more"], ["B (mid)", "about 10 km/h"], ["C (low)", "6 to 8 km/h"]],
               [22 * mm, 40 * mm, 40 * mm, W - 102 * mm]))

s.append(p("D1 &nbsp;Acceleration and coast in gear", H2))
s.append(p("Gives the inertia of each gear, and rolling plus driveline drag.", GIVES))
s.append(steps([
    "If the stretch is quiet, set <b>regenmax = 0</b> for D1 so the coast is drag only. Lifting off then gives "
    "no braking: use the foot brake as normal, but note any braking during a coast. If you leave regen on, "
    "write ON here: ______",
    "<b>Gear A</b>, rolling slowly (about 500 rpm). Press the pedal to a <b>fixed</b> position and hold it: one "
    "that takes about <b>5 s</b> to reach 1800 rpm. Start at about <b>20 %</b>.",
    "At about <b>1800 rpm</b>, release fully. Stay in gear with the clutch engaged and coast down to about "
    "600 rpm. Then drive on normally.",
    "Twice each way along the same stretch (4 runs). Then the same in <b>gear B</b>, starting at about "
    "<b>5 %</b>: lower gears are much lighter. If 1800 rpm comes in under 3 s, use less pedal.",
]))
s.append(Spacer(1, 3))
s.append(table(["Run", "Clock time", "Direction", "Pedal %", "Notes (braking, traffic)"],
               ["D1 A1", "D1 A2", "D1 A3", "D1 A4", "D1 B1", "D1 B2", "D1 B3", "D1 B4"],
               [16 * mm, 22 * mm, 24 * mm, 18 * mm, W - 80 * mm]))

s.append(PageBreak())
s.append(p("D2 &nbsp;Hit the limit in gear", H2))
s.append(p("Overshoot and hunting at three inertias. The harness and bench repeat each run with the logged gains "
           "and pedal.", GIVES))
s.append(boxed([
    p("<b>Set back first:</b> regenmax to its usual value. Gains as you normally run them (revlim 2200, "
      "govKp 11, govKi 0, govKd 0, govFlt 4). Different? Write here: ______________________", BODY),
    Spacer(1, 3),
    p("<b>What the model predicts</b> at these gains (it gives the full requested torque, which the inverter "
      "did not in neutral, so the real tractor should be milder): gear A, a little overshoot that settles; "
      "gear B, a few hundred rpm over and a short ring; gear C, it may <b>surge back and forth</b> at the limit. "
      "That surge is the prediction under test. The model also ignores tyre and shaft wind-up, which this "
      "run checks.", SMALL),
]))
s.append(Spacer(1, 4))
s.append(steps([
    "On the flat stretch, <b>gear A</b>, from about 1200 rpm: press the pedal <b>fully down</b> in one quick "
    "movement and hold it at the limit for about <b>10 s</b>. Then lift off.",
    "<b>Gear B</b>: the same, but step quickly to about <b>25 %</b>, not full.",
    "<b>Gear C</b>: the same, but step to about <b>12 %</b>.",
    "<b>Abort:</b> lift off at once if speed passes <b>2700 rpm</b>, or if the tractor surges hard. Note it. "
    "A surge in gear C is a result, so don't repeat that gear at more pedal.",
    "Three times in each gear, alternating direction.",
]))
s.append(Spacer(1, 3))
s.append(table(["Run", "Clock time", "Pedal %", "Peak rpm", "Notes (surge? how many swings?)"],
               ["D2 A1", "D2 A2", "D2 A3", "D2 B1", "D2 B2", "D2 B3", "D2 C1", "D2 C2", "D2 C3"],
               [16 * mm, 22 * mm, 18 * mm, 20 * mm, W - 76 * mm]))

s.append(p("D3 &nbsp;Governed on a gradient", H2))
s.append(p("Droop on a climb and regen on a descent at the limit: the real version of the bench road sweep.",
           GIVES))
s.append(steps([
    "<b>Gear A</b>, pedal fully down, held at the limit up the steady hill for at least <b>20 s</b>.",
    "Turn round. Gear A, pedal fully down, at the limit down the same hill for at least 20 s. The governor has "
    "to regen to hold speed. Brake if it runs away, and note it.",
    "Twice each way.",
]))
s.append(Spacer(1, 3))
s.append(table(["Run", "Clock time", "Up / down", "Highest / lowest rpm", "Notes"],
               ["D3 #1", "D3 #2", "D3 #3", "D3 #4"],
               [16 * mm, 22 * mm, 20 * mm, 34 * mm, W - 92 * mm]))

s.append(PageBreak())
s.append(p("Optional, parked: R4 again (inverter item V9)", H2))
s.append(p("Only with a logger shown to keep every frame. In neutral, PTO out, revlim 2200: R4 from the R1/R2/R4 "
           "sheet, stepping to about <b>6 %</b>, three times, releasing at once above 2700 rpm.", BODY))

s.append(p("Afterwards", H2))
s.append(checklist([
    "All settings back to the values written on page 1. Saved.",
    "GovLog: leave on / turn off (circle one). DC-DC still fine?",
    "Copy the session files off the logger. Put the clock times, gears, km/h and notes with them (a photo of "
    "this sheet is fine).",
    "On the PC: <font name='Courier'>python tools/loglib.py session_XXXX.csv</font>: the 0x1D4 and 0x4A1 "
    "gap counts should be near 0, and dropped 0x4A1 frames 0.",
]))

s.append(p("What happens to the logs", H2))
s.append(p("D1 gives the inertia of each gear and the road drag. Those replace the guesses in the bench road sweep "
           "(J 2.5, rolling 20 Nm), and they check the gear speeds written above. D2 is the reference for "
           "overshoot and hunting in each gear: the harness and the bench repeat each run with the logged gains "
           "and pedal (validation step 2 at three inertias). That tests the model's central claim, that the "
           "usable govKp scales with inertia, before it is used for PTO work. D3 gives the load on the gradient, "
           "and a profile for the bench. The inertia fit works even on bursty logs. The D2 and D3 traces need a "
           "logger that keeps every frame.", SMALL))

s.append(p("Notes", H2))
s.append(notes(8))

build(OUT, s, "Tractor road runs D1, D2 and D3",
      "Leyland 255 governor work - tractor road runs D1, D2, D3 (SIMULATOR_PLAN.md section 9, milestone M7)")
