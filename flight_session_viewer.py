#!/usr/bin/env python3
"""
Flight Session Viewer (GUI) — parse UAV log.txt files, browse arm sessions,
copy to clipboard or export.

Usage:
    python3 flight_session_viewer.py                     # opens file dialog
    python3 flight_session_viewer.py log.txt [log2.txt]  # opens with files
"""

import sys
import os
import re
import subprocess
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
from dataclasses import dataclass, field
from typing import List, Optional

# ─── Parsing ──────────────────────────────────────────────────────────

TELEM_RE = re.compile(
    r'^(\d+)\s+(\w+)\s+M(\d+)\s+(d|A)(\w)\s+'
    r'R([\d.e+-]+)\s+P([\d.e+-]+)\s+Y([\d.e+-]+)\s+'
)
STATE_RE = re.compile(r'^\[?[\d.]*\]?\s*STATE:\s+(\w+)\s*->\s*(\w+)\s+reason=(.*)')
STATE_BANNER_RE = re.compile(r'>>>\s+STATE:')
BATT_RE = re.compile(r'B\(([\d.]+)V')
ALT_RE = re.compile(r'Alt([\d.e+-]+)\(')
ARMED_BANNER_RE = re.compile(r'^---\s+ARMED\s+t=(\d+)ms\s+---')
DISARMED_BANNER_RE = re.compile(r'^---\s+DISARMED\s+t=(\d+)ms\s+---')
LANDING_REASON_RE = re.compile(r'STATE:.*->\s*LANDING\s+reason=(.*)')
FAILSAFE_RE = re.compile(r'LANDING_FAILSAFE.*reason=(\w+)')


@dataclass
class ArmSession:
    source_file: str
    lines: List[str] = field(default_factory=list)
    start_line: int = 0
    end_line: int = 0
    arm_ts_ms: Optional[int] = None
    disarm_ts_ms: Optional[int] = None
    armed_ts_ms: Optional[int] = None
    duration_ms: Optional[int] = None
    max_alt_m: float = 0.0
    start_batt_v: Optional[float] = None
    end_batt_v: Optional[float] = None
    landing_reason: str = ""
    states_visited: List[str] = field(default_factory=list)

    @property
    def sort_key(self):
        return self.arm_ts_ms or 0


def parse_sessions(filepath: str) -> List[ArmSession]:
    try:
        with open(filepath, 'r', errors='replace') as f:
            all_lines = f.readlines()
    except (IOError, OSError):
        return []

    raw_lines = [l.rstrip('\n\r') for l in all_lines]
    sessions: List[ArmSession] = []
    n = len(raw_lines)

    arm_indices = []
    for i, line in enumerate(raw_lines):
        if STATE_BANNER_RE.search(line):
            continue
        m = STATE_RE.search(line)
        if m and m.group(1) == 'IDLE' and m.group(2) == 'ARMING':
            arm_indices.append(i)

    if not arm_indices:
        return []

    for ai, arm_idx in enumerate(arm_indices):
        start = arm_idx
        for j in range(arm_idx - 1, -1, -1):
            tm = TELEM_RE.match(raw_lines[j])
            if tm:
                if tm.group(2) == 'IDLE':
                    start = j
                else:
                    break
            else:
                start = j
        if ai > 0:
            prev_end = sessions[-1].end_line
            if start <= prev_end:
                start = prev_end + 1

        disarm_idle_idx = None
        for j in range(arm_idx + 1, n):
            if STATE_BANNER_RE.search(raw_lines[j]):
                continue
            m = STATE_RE.search(raw_lines[j])
            if m and m.group(1) == 'DISARMING' and m.group(2) == 'IDLE':
                disarm_idle_idx = j
                break

        if disarm_idle_idx is not None:
            end = disarm_idle_idx
            for j in range(disarm_idle_idx + 1, n):
                tm = TELEM_RE.match(raw_lines[j])
                if tm:
                    if tm.group(2) == 'IDLE':
                        end = j
                    else:
                        break
                elif not STATE_BANNER_RE.search(raw_lines[j]) and STATE_RE.search(raw_lines[j]):
                    m2 = STATE_RE.search(raw_lines[j])
                    if m2 and m2.group(2) == 'ARMING':
                        break
                    end = j
                else:
                    end = j
        else:
            if ai + 1 < len(arm_indices):
                end = arm_indices[ai + 1] - 1
            else:
                end = n - 1

        session = ArmSession(
            source_file=filepath,
            lines=raw_lines[start:end + 1],
            start_line=start,
            end_line=end,
        )

        first_batt = None
        last_batt = None
        states_seen_order = []
        states_seen_set = set()

        for line in session.lines:
            tm = TELEM_RE.match(line)
            if tm:
                state = tm.group(2)
                if state not in states_seen_set:
                    states_seen_set.add(state)
                    states_seen_order.append(state)
                am = ALT_RE.search(line)
                if am:
                    try:
                        alt = float(am.group(1))
                        if alt > session.max_alt_m:
                            session.max_alt_m = alt
                    except ValueError:
                        pass
                bm = BATT_RE.search(line)
                if bm:
                    v = float(bm.group(1))
                    if first_batt is None:
                        first_batt = v
                    last_batt = v

            if not STATE_BANNER_RE.search(line):
                sm = STATE_RE.search(line)
                if sm:
                    st = sm.group(2)
                    if st not in states_seen_set:
                        states_seen_set.add(st)
                        states_seen_order.append(st)

            ab = ARMED_BANNER_RE.search(line)
            if ab:
                session.armed_ts_ms = int(ab.group(1))
            db = DISARMED_BANNER_RE.search(line)
            if db:
                session.disarm_ts_ms = int(db.group(1))

            if not STATE_BANNER_RE.search(line):
                lr = LANDING_REASON_RE.search(line)
                if lr:
                    session.landing_reason = lr.group(1).strip().rstrip('<').strip()
                if not session.landing_reason:
                    sm2 = STATE_RE.search(line)
                    if sm2 and sm2.group(2) == 'DISARMING' and sm2.group(1) != 'LANDING':
                        session.landing_reason = sm2.group(3).strip().rstrip('<').strip()

            ff = FAILSAFE_RE.search(line)
            if ff and not session.landing_reason:
                session.landing_reason = ff.group(1).strip()

        for line in session.lines:
            tm = TELEM_RE.match(line)
            if tm and tm.group(2) == 'ARMING':
                session.arm_ts_ms = int(tm.group(1))
                break
        if session.arm_ts_ms is None and session.armed_ts_ms is not None:
            session.arm_ts_ms = session.armed_ts_ms
        if session.arm_ts_ms is None:
            for line in session.lines:
                tm = TELEM_RE.match(line)
                if tm:
                    session.arm_ts_ms = int(tm.group(1))
                    break

        if session.disarm_ts_ms is None:
            for line in reversed(session.lines):
                tm = TELEM_RE.match(line)
                if tm and tm.group(2) == 'DISARMING':
                    session.disarm_ts_ms = int(tm.group(1))
                    break
        if session.disarm_ts_ms is None:
            for line in reversed(session.lines):
                tm = TELEM_RE.match(line)
                if tm:
                    session.disarm_ts_ms = int(tm.group(1))
                    break

        if session.arm_ts_ms is not None and session.disarm_ts_ms is not None:
            session.duration_ms = session.disarm_ts_ms - session.arm_ts_ms

        session.start_batt_v = first_batt
        session.end_batt_v = last_batt
        session.states_visited = states_seen_order
        sessions.append(session)

    return sessions


def format_duration(ms: Optional[int]) -> str:
    if ms is None:
        return "?"
    s = ms / 1000.0
    if s < 60:
        return f"{s:.1f}s"
    m = int(s // 60)
    s = s % 60
    return f"{m}m{s:.0f}s"


def copy_to_clipboard_system(text: str) -> bool:
    for cmd in (['/mnt/c/Windows/System32/clip.exe'],
                ['clip.exe'],
                ['xclip', '-selection', 'clipboard'],
                ['pbcopy']):
        try:
            p = subprocess.run(cmd, input=text.encode('utf-8', errors='replace'),
                               capture_output=True, timeout=5)
            if p.returncode == 0:
                return True
        except (FileNotFoundError, subprocess.TimeoutExpired):
            continue
    return False


# ─── GUI ──────────────────────────────────────────────────────────────

# Color scheme
BG           = "#1e1e2e"
BG_SURFACE   = "#252536"
BG_CARD      = "#2a2a3d"
BG_HOVER     = "#33334a"
FG           = "#cdd6f4"
FG_DIM       = "#7f849c"
FG_BRIGHT    = "#ffffff"
ACCENT       = "#89b4fa"
ACCENT_HOVER = "#74a8fc"
GREEN        = "#a6e3a1"
YELLOW       = "#f9e2af"
RED          = "#f38ba8"
PEACH        = "#fab387"
BORDER       = "#45475a"
SELECT_BG    = "#313244"


class App(tk.Tk):
    def __init__(self, initial_files: List[str] = None):
        super().__init__()
        self.title("Flight Session Viewer")
        self.geometry("1100x720")
        self.minsize(800, 500)
        self.configure(bg=BG)

        self.sessions: List[ArmSession] = []
        self.loaded_files: List[str] = []
        self.sort_col = None
        self.sort_reverse = True  # default: newest first

        self._build_styles()
        self._build_ui()

        # Keybinds
        self.bind("<Control-o>", lambda e: self._on_open())
        self.bind("<Control-c>", lambda e: self._on_copy())

        if initial_files:
            self.after(100, lambda: self._load_files(initial_files))

    # ── Styles ────────────────────────────────────────────────────────

    def _build_styles(self):
        style = ttk.Style(self)
        style.theme_use("clam")

        # Treeview
        style.configure("Session.Treeview",
                         background=BG_SURFACE,
                         foreground=FG,
                         fieldbackground=BG_SURFACE,
                         borderwidth=0,
                         rowheight=26,
                         font=("Consolas", 10))
        style.configure("Session.Treeview.Heading",
                         background=BG_CARD,
                         foreground=ACCENT,
                         borderwidth=1,
                         relief="flat",
                         font=("Segoe UI", 9, "bold"))
        style.map("Session.Treeview",
                   background=[("selected", SELECT_BG)],
                   foreground=[("selected", FG_BRIGHT)])
        style.map("Session.Treeview.Heading",
                   background=[("active", BG_HOVER)])

        # Scrollbar
        style.configure("Dark.Vertical.TScrollbar",
                         background=BG_CARD,
                         troughcolor=BG_SURFACE,
                         borderwidth=0,
                         arrowsize=12)
        style.map("Dark.Vertical.TScrollbar",
                   background=[("active", BORDER)])

    # ── Layout ────────────────────────────────────────────────────────

    def _build_ui(self):
        # ── Toolbar ──
        toolbar = tk.Frame(self, bg=BG, pady=6, padx=10)
        toolbar.pack(fill=tk.X)

        self.btn_open = tk.Button(
            toolbar, text="Open Log Files",
            command=self._on_open,
            bg=ACCENT, fg=BG, activebackground=ACCENT_HOVER, activeforeground=BG,
            font=("Segoe UI", 10, "bold"), bd=0, padx=14, pady=4,
            cursor="hand2", relief="flat")
        self.btn_open.pack(side=tk.LEFT)

        self.lbl_files = tk.Label(
            toolbar, text="  No files loaded", anchor="w",
            bg=BG, fg=FG_DIM, font=("Segoe UI", 9))
        self.lbl_files.pack(side=tk.LEFT, padx=(12, 0), fill=tk.X, expand=True)

        self.lbl_count = tk.Label(
            toolbar, text="", anchor="e",
            bg=BG, fg=FG_DIM, font=("Segoe UI", 9))
        self.lbl_count.pack(side=tk.RIGHT)

        # ── Main paned window (top = session list, bottom = log viewer) ──
        self.pane = tk.PanedWindow(
            self, orient=tk.VERTICAL, bg=BORDER,
            sashwidth=4, sashrelief="flat", bd=0)
        self.pane.pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 8))

        # ── Top: session list ──
        top_frame = tk.Frame(self.pane, bg=BG_SURFACE)

        columns = ("num", "arm_t", "duration", "altitude", "battery",
                    "phases", "end_reason", "file")
        self.tree = ttk.Treeview(
            top_frame, columns=columns, show="headings",
            style="Session.Treeview", selectmode="browse")

        headings = {
            "num":        ("#",         40,  tk.CENTER),
            "arm_t":      ("Arm Time",  90,  tk.CENTER),
            "duration":   ("Duration",  70,  tk.CENTER),
            "altitude":   ("Max Alt",   70,  tk.CENTER),
            "battery":    ("Battery",   130, tk.CENTER),
            "phases":     ("Phases",    230, tk.W),
            "end_reason": ("End Reason",140, tk.W),
            "file":       ("File",      160, tk.W),
        }
        for col, (text, width, anchor) in headings.items():
            self.tree.heading(col, text=text,
                              command=lambda c=col: self._on_sort(c))
            self.tree.column(col, width=width, anchor=anchor, minwidth=40)

        tree_scroll = ttk.Scrollbar(
            top_frame, orient=tk.VERTICAL, command=self.tree.yview,
            style="Dark.Vertical.TScrollbar")
        self.tree.configure(yscrollcommand=tree_scroll.set)

        self.tree.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        tree_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.tree.bind("<<TreeviewSelect>>", self._on_select)

        self.pane.add(top_frame, minsize=140, height=300)

        # ── Bottom: detail panel ──
        bottom_frame = tk.Frame(self.pane, bg=BG)

        # Detail header + action buttons
        detail_bar = tk.Frame(bottom_frame, bg=BG, pady=4)
        detail_bar.pack(fill=tk.X)

        self.lbl_detail = tk.Label(
            detail_bar, text="Select a session above",
            bg=BG, fg=FG_DIM, font=("Segoe UI", 9), anchor="w")
        self.lbl_detail.pack(side=tk.LEFT)

        btn_style = dict(
            bd=0, padx=12, pady=3, cursor="hand2", relief="flat",
            font=("Segoe UI", 9, "bold"))

        self.btn_export = tk.Button(
            detail_bar, text="Export .txt",
            command=self._on_export,
            bg=BG_CARD, fg=PEACH,
            activebackground=BG_HOVER, activeforeground=PEACH,
            **btn_style)
        self.btn_export.pack(side=tk.RIGHT, padx=(4, 0))

        self.btn_copy = tk.Button(
            detail_bar, text="Copy to Clipboard",
            command=self._on_copy,
            bg=BG_CARD, fg=GREEN,
            activebackground=BG_HOVER, activeforeground=GREEN,
            **btn_style)
        self.btn_copy.pack(side=tk.RIGHT, padx=(4, 0))

        # Log text area
        text_frame = tk.Frame(bottom_frame, bg=BORDER, bd=1, relief="flat")
        text_frame.pack(fill=tk.BOTH, expand=True)

        self.text = tk.Text(
            text_frame, wrap=tk.NONE, state=tk.DISABLED,
            bg=BG_SURFACE, fg=FG, insertbackground=FG,
            font=("Consolas", 9), bd=0, padx=6, pady=4,
            selectbackground=SELECT_BG, selectforeground=FG_BRIGHT)

        text_yscroll = tk.Scrollbar(
            text_frame, orient=tk.VERTICAL, command=self.text.yview,
            bg=BG_CARD, troughcolor=BG_SURFACE, bd=0)
        text_xscroll = tk.Scrollbar(
            text_frame, orient=tk.HORIZONTAL, command=self.text.xview,
            bg=BG_CARD, troughcolor=BG_SURFACE, bd=0)
        self.text.configure(yscrollcommand=text_yscroll.set,
                            xscrollcommand=text_xscroll.set)

        text_yscroll.pack(side=tk.RIGHT, fill=tk.Y)
        text_xscroll.pack(side=tk.BOTTOM, fill=tk.X)
        self.text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Configure text tags for syntax highlighting
        self.text.tag_configure("state_line", foreground=ACCENT)
        self.text.tag_configure("event_line", foreground=YELLOW)
        self.text.tag_configure("fail_line", foreground=RED)
        self.text.tag_configure("armed_banner", foreground=GREEN)
        self.text.tag_configure("disarmed_banner", foreground=PEACH)
        self.text.tag_configure("telem_idle", foreground=FG_DIM)

        self.pane.add(bottom_frame, minsize=120)

        # ── Status bar ──
        self.status = tk.Label(
            self, text="", bg=BG, fg=FG_DIM,
            font=("Segoe UI", 8), anchor="w", padx=10, pady=2)
        self.status.pack(fill=tk.X, side=tk.BOTTOM)

    # ── Actions ───────────────────────────────────────────────────────

    def _on_open(self):
        filetypes = [("Log files", "*.txt"), ("All files", "*.*")]
        paths = filedialog.askopenfilenames(
            title="Open UAV Log Files", filetypes=filetypes)
        if paths:
            self._load_files(list(paths))

    def _load_files(self, paths: List[str]):
        self.sessions.clear()
        self.loaded_files.clear()
        errors = []

        for p in paths:
            if not os.path.isfile(p):
                continue
            found = parse_sessions(p)
            if found:
                self.loaded_files.append(p)
                self.sessions.extend(found)
            else:
                errors.append(os.path.basename(p))

        # Sort newest first by file mtime + arm timestamp
        file_mtimes = {}
        for s in self.sessions:
            if s.source_file not in file_mtimes:
                try:
                    file_mtimes[s.source_file] = os.path.getmtime(s.source_file)
                except OSError:
                    file_mtimes[s.source_file] = 0
        self.sessions.sort(
            key=lambda s: (file_mtimes.get(s.source_file, 0), s.sort_key),
            reverse=True)

        self.sort_col = None
        self.sort_reverse = True

        self._refresh_tree()

        names = [os.path.basename(p) for p in self.loaded_files]
        self.lbl_files.config(text="  " + ", ".join(names) if names else "  No files loaded")
        self.lbl_count.config(text=f"{len(self.sessions)} session(s)")

        if errors:
            self._set_status(f"No sessions in: {', '.join(errors)}")
        else:
            self._set_status(f"Loaded {len(self.sessions)} session(s) from {len(self.loaded_files)} file(s)")

    def _refresh_tree(self):
        self.tree.delete(*self.tree.get_children())
        show_file = len(set(s.source_file for s in self.sessions)) > 1

        for i, s in enumerate(self.sessions):
            dur = format_duration(s.duration_ms)
            alt = f"{s.max_alt_m:.2f}m" if s.max_alt_m > 0 else "gnd"
            batt = ""
            if s.start_batt_v and s.end_batt_v:
                batt = f"{s.start_batt_v:.2f} -> {s.end_batt_v:.2f}V"
            elif s.start_batt_v:
                batt = f"{s.start_batt_v:.2f}V"
            phases_list = [p for p in s.states_visited
                           if p not in ('IDLE', 'ARMING', 'DISARMING')]
            phases = " -> ".join(phases_list) if phases_list else "no-flight"
            reason = s.landing_reason or "normal"
            arm_t = f"{s.arm_ts_ms}ms" if s.arm_ts_ms else "?"
            fname = os.path.basename(s.source_file) if show_file else ""

            tag = ""
            if "FAIL" in reason.upper() or "DRIFT" in reason.upper() or "STALL" in reason.upper():
                tag = "fail"
            elif "ABORT" in reason.upper() or "TIMEOUT" in reason.upper():
                tag = "warn"

            self.tree.insert("", tk.END, iid=str(i),
                             values=(i + 1, arm_t, dur, alt, batt, phases, reason, fname),
                             tags=(tag,))

        self.tree.tag_configure("fail", foreground=RED)
        self.tree.tag_configure("warn", foreground=YELLOW)

        # Clear detail
        self.text.config(state=tk.NORMAL)
        self.text.delete("1.0", tk.END)
        self.text.config(state=tk.DISABLED)
        self.lbl_detail.config(text="Select a session above")

    def _on_select(self, _event=None):
        sel = self.tree.selection()
        if not sel:
            return
        idx = int(sel[0])
        s = self.sessions[idx]

        dur = format_duration(s.duration_ms)
        reason = s.landing_reason or "normal"
        self.lbl_detail.config(
            text=f"Session #{idx+1}  |  {len(s.lines)} lines  |  "
                 f"Duration: {dur}  |  Max alt: {s.max_alt_m:.2f}m  |  "
                 f"End: {reason}  |  "
                 f"{os.path.basename(s.source_file)} "
                 f"(lines {s.start_line+1}-{s.end_line+1})",
            fg=FG)

        self.text.config(state=tk.NORMAL)
        self.text.delete("1.0", tk.END)

        for line in s.lines:
            tag = self._classify_line(line)
            self.text.insert(tk.END, line + "\n", tag)

        self.text.config(state=tk.DISABLED)
        self.text.yview_moveto(0)

    def _classify_line(self, line: str) -> str:
        if STATE_BANNER_RE.search(line):
            return "state_line"
        if STATE_RE.search(line):
            return "state_line"
        if ARMED_BANNER_RE.search(line):
            return "armed_banner"
        if DISARMED_BANNER_RE.search(line):
            return "disarmed_banner"
        if "FAILSAFE" in line or "FAIL" in line or "PRE-FAIL" in line:
            return "fail_line"
        if "EVENT:" in line:
            return "event_line"
        tm = TELEM_RE.match(line)
        if tm and tm.group(2) == 'IDLE':
            return "telem_idle"
        return ""

    def _on_sort(self, col):
        if self.sort_col == col:
            self.sort_reverse = not self.sort_reverse
        else:
            self.sort_col = col
            self.sort_reverse = True  # default descending for most columns

        key_map = {
            "num":        lambda s: 0,
            "arm_t":      lambda s: s.arm_ts_ms or 0,
            "duration":   lambda s: s.duration_ms or 0,
            "altitude":   lambda s: s.max_alt_m,
            "battery":    lambda s: s.start_batt_v or 0,
            "phases":     lambda s: " -> ".join(s.states_visited),
            "end_reason": lambda s: s.landing_reason,
            "file":       lambda s: os.path.basename(s.source_file),
        }
        key_fn = key_map.get(col, lambda s: 0)
        self.sessions.sort(key=key_fn, reverse=self.sort_reverse)
        self._refresh_tree()

        # Update heading arrows
        for c in self.tree["columns"]:
            text = self.tree.heading(c, "text").rstrip(" \u25b2\u25bc")
            if c == col:
                arrow = " \u25bc" if self.sort_reverse else " \u25b2"
                text += arrow
            self.tree.heading(c, text=text)

    def _selected_session(self) -> Optional[ArmSession]:
        sel = self.tree.selection()
        if not sel:
            self._set_status("No session selected")
            return None
        return self.sessions[int(sel[0])]

    def _on_copy(self):
        s = self._selected_session()
        if not s:
            return
        text = '\n'.join(s.lines) + '\n'
        # Try system clipboard first (works better in WSL), fall back to tk
        if copy_to_clipboard_system(text):
            self._set_status(f"Copied {len(s.lines)} lines to clipboard")
        else:
            self.clipboard_clear()
            self.clipboard_append(text)
            self._set_status(f"Copied {len(s.lines)} lines to clipboard (tk)")

    def _on_export(self):
        s = self._selected_session()
        if not s:
            return
        default_name = (
            f"session_{s.arm_ts_ms or 'unknown'}ms_"
            f"{os.path.splitext(os.path.basename(s.source_file))[0]}.txt")
        path = filedialog.asksaveasfilename(
            title="Export Session",
            defaultextension=".txt",
            initialfile=default_name,
            filetypes=[("Text files", "*.txt"), ("All files", "*.*")])
        if not path:
            return
        try:
            with open(path, 'w') as f:
                f.write('\n'.join(s.lines) + '\n')
            self._set_status(f"Exported to {path}")
        except IOError as e:
            messagebox.showerror("Export Error", str(e))

    def _set_status(self, msg: str):
        self.status.config(text=msg)
        self.after(8000, lambda: self.status.config(text=""))


# ─── Entry point ──────────────────────────────────────────────────────

def main():
    initial_files = [f for f in sys.argv[1:] if not f.startswith('-')]
    app = App(initial_files=initial_files if initial_files else None)
    app.mainloop()


if __name__ == '__main__':
    main()
