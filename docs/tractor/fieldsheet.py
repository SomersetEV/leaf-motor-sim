"""Shared layout for the printable tractor field sheets (make_*_sheet.py)."""
from reportlab.lib import colors
from reportlab.lib.enums import TA_LEFT
from reportlab.lib.pagesizes import A4
from reportlab.lib.styles import ParagraphStyle, getSampleStyleSheet
from reportlab.lib.units import mm
from reportlab.platypus import (Flowable, PageBreak, Paragraph, SimpleDocTemplate, Spacer,
                                Table, TableStyle)


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



def table(head, rows, widths):
    """A ruled table to fill in by hand. head is the column titles; each row is its first
    cell, or a list of its leading cells."""
    data = [[p(h, CELLB) for h in head]]
    for r in rows:
        cells = [p(c, CELL) for c in ([r] if isinstance(r, str) else r)]
        data.append(cells + [""] * (len(head) - len(cells)))
    t = Table(data, colWidths=widths, rowHeights=[None] + [10 * mm] * len(rows))
    t.setStyle(TableStyle([
        ("GRID", (0, 0), (-1, -1), 0.5, colors.black),
        ("BACKGROUND", (0, 0), (-1, 0), colors.HexColor("#E6E6E6")),
        ("VALIGN", (0, 0), (-1, -1), "MIDDLE"),
    ]))
    return t


def notes(n=6):
    lines = Table([[""]] * n, colWidths=[W], rowHeights=[8 * mm] * n)
    lines.setStyle(TableStyle([("LINEBELOW", (0, 0), (-1, -1), 0.4, colors.HexColor("#888888"))]))
    return lines


def build(out, story, title, footer_text):
    """Writes the PDF with footer_text and 'Page n of N' on every page."""
    def footer(canvas, doc, total):
        canvas.saveState()
        canvas.setFont("Helvetica", 7.5)
        canvas.setFillColor(colors.HexColor("#555555"))
        canvas.drawString(16 * mm, 10 * mm, footer_text)
        canvas.drawRightString(A4[0] - 16 * mm, 10 * mm, f"Page {doc.page} of {total}")
        canvas.restoreState()

    def make(total):
        d = SimpleDocTemplate(out, pagesize=A4, leftMargin=16 * mm, rightMargin=16 * mm, topMargin=14 * mm,
                bottomMargin=16 * mm, title=title, author="Ben Bament")
        f = lambda c, dd: footer(c, dd, total)  # noqa: E731
        d.build(list(story), onFirstPage=f, onLaterPages=f)
        return d.page

    pages = make("?")
    make(pages)  # second pass, now that the page count is known
    print("written", out)
