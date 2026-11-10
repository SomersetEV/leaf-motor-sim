"""Printable A4 field sheet for tractor captures R1, R2 and R4 (plan section 9, M7).

    python make_field_sheet.py [OUT.pdf]

Writes M7_R1_R2_R4_field_sheet.pdf next to this script by default. Needs
reportlab, which is not in tools/requirements.txt (pip install reportlab).
"""
import sys
from pathlib import Path

from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Flowable, PageBreak, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)

OUT = sys.argv[1] if len(sys.argv) > 1 else str(Path(__file__).resolve().parent / "M7_R1_R2_R4_field_sheet.pdf")

ss = getSampleStyleSheet()
BODY = ParagraphStyle("body", parent=ss["Normal"], fontName="Helvetica", fontSize=9.5, leading=12.5)
SMALL = ParagraphStyle("small", parent=BODY, fontSize=8.5, leading=11, textColor=colors.HexColor("#333333"))
H1 = ParagraphStyle("h1", parent=BODY, fontName="Helvetica-Bold", fontSize=16, leading=20, spaceAfter=2)
H2 = ParagraphStyle("h2", parent=BODY, fontName="Helvetica-Bold", fontSize=12, leading=15,
                    spaceBefore=8, spaceAfter=3)
GIVES = ParagraphStyle("gives", parent=BODY, fontName="Helvetica-Oblique", fontSize=9, leading=12,
                       spaceAfter=3)
CELL = ParagraphStyle("cell", parent=BODY, fontSize=8.5, leading=10.5, alignment=TA_LEFT)
CELLB = ParagraphStyle("cellb", parent=CELL, fontName="Helvetica-Bold")

W = A4[0] - 32 * mm


class TickBox(Flowable):
    def __init__(self, size=3.4 * mm):
        super().__init__()
        self.size = size

    def wrap(self, aw, ah):
        return self.size, self.size + 1

    def draw(self):
        self.canv.setLineWidth(0.8)
        self.canv.rect(0, 0.6, self.size, self.size, stroke=1, fill=0)


def p(text, style=BODY):
    return Paragraph(text, style)


def checklist(items, style=BODY, width=W):
    rows = [[TickBox(), p(t, style)] for t in items]
    t = Table(rows, colWidths=[6 * mm, width - 6 * mm])
    t.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("RIGHTPADDING", (0, 0), (-1, -1), 2),
        ("TOPPADDING", (0, 0), (-1, -1), 1.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5),
    ]))
    return t


def steps(items):
    rows = [[p(f"<b>{i}</b>"), p(t)] for i, t in enumerate(items, 1)]
    t = Table(rows, colWidths=[6 * mm, W - 6 * mm])
    t.setStyle(TableStyle([
        ("VALIGN", (0, 0), (-1, -1), "TOP"),
        ("LEFTPADDING", (0, 0), (-1, -1), 0),
        ("TOPPADDING", (0, 0), (-1, -1), 1.5),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 1.5),
    ]))
    return t


def record(run, rows, extra_col="Pedal %", rpm_col="Release rpm"):
    head = ["Run", "Clock time", "Session file", extra_col, rpm_col, "Notes"]
    data = [[p(h, CELLB) for h in head]]
    for r in rows:
        data.append([p(r, CELL), "", "", "", "", ""])
    widths = [21 * mm, 20 * mm, 27 * mm, 20 * mm, 22 * mm, W - 110 * mm]
    t = Table(data, colWidths=widths, rowHeights=[None] + [11 * mm] * len(rows))
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.5, colors.black),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#E6E6E6")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ]))
    return t


def boxed(flowables):
    t = Table([[flowables]], colWidths=[W])
    t.setStyle(TableStyle([
        ("BOX", (0, 0), (-1, -1), 0.8, colors.black),
        ("LEFTPADDING", (0, 0), (-1, -1), 5),
        ("RIGHTPADDING", (0, 0), (-1, -1), 5),
        ("TOPPADDING", (0, 0), (-1, -1), 4),
        ("BOTTOMPADDING", (0, 0), (-1, -1), 5),
    ]))
    return t


def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont("Helvetica", 7.5)
    canvas.setFillColor(colors.HexColor("#555555"))
    canvas.drawString(16 * mm, 10 * mm, "Leyland 255 governor work - tractor captures R1, R2, R4 "
                      "(SIMULATOR_PLAN.md section 9, milestone M7)")
    canvas.drawRightString(A4[0] - 16 * mm, 10 * mm, f"Page {doc.page} of 2")
    canvas.restoreState()


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
s.append(p("Gives the inertia J of the motor and gearbox input.", GIVES))
s.append(steps([
    "Set <b>revlim = 3000</b> so the governor stays out of the way.",
    "From standstill, press the throttle to a <b>fixed, low</b> position and hold it still. Aim for one that "
    "takes about <b>3 to 5 s</b> to reach 2000 rpm: start at about 10 to 20 % and adjust. A block or stop "
    "under the pedal helps keep it steady.",
    "At about <b>2000 rpm</b> release the throttle fully and let it coast to a stop. That coast is R2 "
    "(next section), so let it run down without touching anything.",
    "Repeat <b>three times</b>, same pedal position each time. Note the pedal % you used.",
]))
s.append(Spacer(1, 4))
s.append(record("R1", ["R1 / R2 #1", "R1 / R2 #2", "R1 / R2 #3"]))

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
]))
s.append(Spacer(1, 4))
s.append(steps([
    "Still parked, neutral, PTO out. Start from <b>standstill</b>, throttle closed.",
    "<b>Snap</b> the throttle fully open in one quick movement, and hold it there.",
    "Keep it held until speed has <b>settled at the limit</b> for about 5 s, then release and let it stop.",
    "Repeat <b>three times</b>. Wait for the shaft to stop between repeats.",
]))
s.append(Spacer(1, 4))
s.append(record("R4", ["R4 #1", "R4 #2", "R4 #3"], extra_col="Gains as above?", rpm_col="Peak rpm"))

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
           "the gains recorded in the log (step 2: overshoot within 20 % or 30 rpm, whichever is larger, and "
           "similar dip-and-creep). Only once those match are bench sweeps trusted for tuning.", SMALL))

s.append(p("Notes", H2))
lines = Table([[""]] * 6, colWidths=[W], rowHeights=[8 * mm] * 6)
lines.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, -1), 0.4, colors.HexColor("#888888"))]))
s.append(lines)

doc = SimpleDocTemplate(OUT, pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm,
                        topMargin=14 * mm, bottomMargin=16 * mm,
                        title="Tractor captures R1, R2 and R4", author="Ben Bament")
doc.build(s, onFirstPage=footer, onLaterPages=footer)
print("written", OUT)
