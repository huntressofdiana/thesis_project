"""
memory_schedule_model.py

Deterministic reference model for the thesis DDR4 buffering/scheduling path.

Purpose
-------
This is NOT a model of MIG internals or DDR4 electrical timing.

It is an independent behavioural model of what the design is EXPECTED to do
at the memory-side interface:

    packet/frame data arrives
        -> data is written to an even/odd DDR buffer
        -> after a fixed delay, data is read back
        -> reads/writes occur only in deterministic time slots

The model generates a cycle-by-cycle CSV trace that can later be compared
against Vivado behavioural simulation.

Version 1 deliberately uses synthetic frame data.  Later, the synthetic input
can be replaced by the packetizer test data supplied by the supervisor.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
import csv
from typing import Dict, List, Optional, Tuple


# =============================================================================
# EDIT THESE PARAMETERS FIRST
# =============================================================================

@dataclass(frozen=True)
class Config:
    # Interface width
    DATA_WIDTH_BITS: int = 128
    BYTES_PER_BEAT: int = 16          # 128 bits / 8

    # Small synthetic test case
    NUM_FRAMES: int = 4
    FRAME_BEATS: int = 64             # keep small while debugging

    # Synthetic packetizer/input timing
    # One 128-bit input beat arrives every N model clock cycles.
    # This is a TEST parameter, not yet the final packetizer timing.
    INPUT_BEAT_PERIOD: int = 3
    FRAME_GAP_CYCLES: int = 12

    # Fixed delay from the first beat of a frame arriving until that frame
    # becomes eligible to be read from DDR.
    READ_START_DELAY_CYCLES: int = 120

    # Fixed DDR service schedule:
    #   WRITE_BURST_BEATS write slots
    #   TURNAROUND_GAP_CYCLES idle slots
    #   READ_BURST_BEATS read slots
    #   TURNAROUND_GAP_CYCLES idle slots
    #   repeat
    WRITE_BURST_BEATS: int = 16
    READ_BURST_BEATS: int = 16
    TURNAROUND_GAP_CYCLES: int = 2

    # Two logical frame buffers.  Frame 0,2,4... reuse EVEN_BASE.
    # Frame 1,3,5... reuse ODD_BASE.
    EVEN_BASE: int = 0x0000_0000
    ODD_BASE: int = 0x0100_0000

    # Safety limit so a broken configuration cannot run forever.
    MAX_CYCLES: int = 100_000

    # Output directory
    OUTPUT_DIR: str = "model_output"


CFG = Config()


# =============================================================================
# DATA STRUCTURES
# =============================================================================

@dataclass
class Beat:
    frame: int
    beat: int
    arrival_cycle: int
    address: int
    data: int


@dataclass
class MemoryEntry:
    data: int
    frame: int
    beat: int
    write_cycle: int
    read_cycle: Optional[int] = None

    @property
    def consumed(self) -> bool:
        return self.read_cycle is not None


@dataclass
class FrameStats:
    frame: int
    stream: str
    frame_start_cycle: int
    read_eligible_cycle: int

    first_write: Optional[int] = None
    last_write: Optional[int] = None
    first_read: Optional[int] = None
    last_read: Optional[int] = None

    min_write_to_read_gap: Optional[int] = None
    max_write_to_read_gap: Optional[int] = None


# =============================================================================
# BASIC HELPERS
# =============================================================================

def frame_base(frame: int, cfg: Config = CFG) -> int:
    """Even frames go to EVEN_BASE; odd frames go to ODD_BASE."""
    return cfg.EVEN_BASE if frame % 2 == 0 else cfg.ODD_BASE


def frame_stream(frame: int) -> str:
    return "even" if frame % 2 == 0 else "odd"


def make_data(frame: int, beat: int) -> int:
    """
    Easy-to-recognise 128-bit synthetic data pattern.

    Upper 64 bits = frame number
    Lower 64 bits = beat number
    """
    return ((frame & ((1 << 64) - 1)) << 64) | (beat & ((1 << 64) - 1))


def make_input_beats(cfg: Config = CFG) -> Tuple[List[Beat], Dict[int, FrameStats]]:
    """
    Create a deterministic synthetic packet/frame input stream.

    Frames arrive one after another.  Within a frame, one beat arrives every
    INPUT_BEAT_PERIOD cycles.
    """
    beats: List[Beat] = []
    stats: Dict[int, FrameStats] = {}

    frame_duration = (cfg.FRAME_BEATS - 1) * cfg.INPUT_BEAT_PERIOD + 1
    frame_spacing = frame_duration + cfg.FRAME_GAP_CYCLES

    for frame in range(cfg.NUM_FRAMES):
        start = frame * frame_spacing
        stats[frame] = FrameStats(
            frame=frame,
            stream=frame_stream(frame),
            frame_start_cycle=start,
            read_eligible_cycle=start + cfg.READ_START_DELAY_CYCLES,
        )

        base = frame_base(frame, cfg)

        for beat in range(cfg.FRAME_BEATS):
            arrival = start + beat * cfg.INPUT_BEAT_PERIOD
            address = base + beat * cfg.BYTES_PER_BEAT
            data = make_data(frame, beat)

            beats.append(
                Beat(
                    frame=frame,
                    beat=beat,
                    arrival_cycle=arrival,
                    address=address,
                    data=data,
                )
            )

    beats.sort(key=lambda x: x.arrival_cycle)
    return beats, stats


def slot_for_cycle(cycle: int, cfg: Config = CFG) -> Tuple[str, int, int]:
    """
    Return:
        slot_type: WRITE_SLOT, READ_SLOT, or GAP
        window_id: repeating schedule-window index
        beat_in_window: position within the active write/read window

    This creates a completely deterministic memory service schedule.
    """
    period = (
        cfg.WRITE_BURST_BEATS
        + cfg.TURNAROUND_GAP_CYCLES
        + cfg.READ_BURST_BEATS
        + cfg.TURNAROUND_GAP_CYCLES
    )

    offset = cycle % period
    window_id = cycle // period

    if offset < cfg.WRITE_BURST_BEATS:
        return "WRITE_SLOT", window_id, offset

    offset -= cfg.WRITE_BURST_BEATS

    if offset < cfg.TURNAROUND_GAP_CYCLES:
        return "GAP", window_id, offset

    offset -= cfg.TURNAROUND_GAP_CYCLES

    if offset < cfg.READ_BURST_BEATS:
        return "READ_SLOT", window_id, offset

    return "GAP", window_id, offset - cfg.READ_BURST_BEATS


# =============================================================================
# REFERENCE MODEL
# =============================================================================

class ReferenceModel:
    def __init__(self, cfg: Config = CFG):
        self.cfg = cfg

        self.input_beats, self.frame_stats = make_input_beats(cfg)
        self.next_input_index = 0

        # Beats that have arrived but have not yet been committed to DDR.
        self.write_queue: List[Beat] = []

        # Address -> latest DDR contents.
        self.memory: Dict[int, MemoryEntry] = {}

        # Next beat that should be read for each frame.
        self.next_read_beat: Dict[int, int] = {
            frame: 0 for frame in range(cfg.NUM_FRAMES)
        }

        self.schedule_rows: List[dict] = []
        self.errors: List[str] = []

        self.max_write_queue_depth = 0
        self.completed_reads = 0
        self.total_beats = cfg.NUM_FRAMES * cfg.FRAME_BEATS

    # -------------------------------------------------------------------------
    # Validation
    # -------------------------------------------------------------------------

    def validate_config(self) -> None:
        c = self.cfg

        if c.DATA_WIDTH_BITS != c.BYTES_PER_BEAT * 8:
            raise ValueError("DATA_WIDTH_BITS must equal BYTES_PER_BEAT * 8")

        if c.DATA_WIDTH_BITS != 128:
            print(
                "WARNING: This thesis model was prepared for a 128-bit interface, "
                f"but DATA_WIDTH_BITS={c.DATA_WIDTH_BITS}."
            )

        if c.BYTES_PER_BEAT & (c.BYTES_PER_BEAT - 1):
            raise ValueError("BYTES_PER_BEAT must be a power of two")

        if c.WRITE_BURST_BEATS <= 0 or c.READ_BURST_BEATS <= 0:
            raise ValueError("Burst lengths must be positive")

        if c.INPUT_BEAT_PERIOD <= 0:
            raise ValueError("INPUT_BEAT_PERIOD must be positive")

    # -------------------------------------------------------------------------
    # Input arrival
    # -------------------------------------------------------------------------

    def accept_arrivals(self, cycle: int) -> List[Beat]:
        """Move all beats arriving on this cycle into the write queue."""
        arrived: List[Beat] = []

        while self.next_input_index < len(self.input_beats):
            beat = self.input_beats[self.next_input_index]

            if beat.arrival_cycle != cycle:
                break

            self.write_queue.append(beat)
            arrived.append(beat)
            self.next_input_index += 1

        self.max_write_queue_depth = max(
            self.max_write_queue_depth, len(self.write_queue)
        )

        return arrived

    # -------------------------------------------------------------------------
    # Memory operations
    # -------------------------------------------------------------------------

    def do_write(self, cycle: int) -> Tuple[str, Optional[Beat], str]:
        """
        Perform one expected 128-bit memory write if data is available.

        Returns (operation, beat, note).
        """
        if not self.write_queue:
            return "IDLE", None, "write slot but no input data available"

        beat = self.write_queue.pop(0)

        if beat.address % self.cfg.BYTES_PER_BEAT != 0:
            self.errors.append(
                f"Cycle {cycle}: unaligned write address 0x{beat.address:08X}"
            )

        old = self.memory.get(beat.address)

        if old is not None and not old.consumed:
            self.errors.append(
                f"Cycle {cycle}: OVERWRITE BEFORE READ at "
                f"0x{beat.address:08X}. "
                f"Frame {beat.frame} beat {beat.beat} overwrites "
                f"frame {old.frame} beat {old.beat}, "
                f"written at cycle {old.write_cycle}."
            )

        self.memory[beat.address] = MemoryEntry(
            data=beat.data,
            frame=beat.frame,
            beat=beat.beat,
            write_cycle=cycle,
        )

        s = self.frame_stats[beat.frame]
        if s.first_write is None:
            s.first_write = cycle
        s.last_write = cycle

        return "WRITE", beat, ""

    def choose_read_candidate(self, cycle: int) -> Optional[Tuple[int, int]]:
        """
        Choose the oldest frame that:
          * has reached its fixed read-eligible time
          * still has unread beats

        Returns (frame, beat_index).
        """
        candidates = []

        for frame, next_beat in self.next_read_beat.items():
            if next_beat >= self.cfg.FRAME_BEATS:
                continue

            s = self.frame_stats[frame]

            if cycle >= s.read_eligible_cycle:
                candidates.append((s.read_eligible_cycle, frame, next_beat))

        if not candidates:
            return None

        candidates.sort()
        _, frame, beat = candidates[0]
        return frame, beat

    def do_read(
        self, cycle: int
    ) -> Tuple[str, Optional[Beat], str, Optional[int]]:
        """
        Perform one expected read.

        Returns:
            operation
            beat
            note
            write_to_read_gap
        """
        candidate = self.choose_read_candidate(cycle)

        if candidate is None:
            return "IDLE", None, "read slot but no frame is eligible", None

        frame, beat_index = candidate
        address = frame_base(frame, self.cfg) + beat_index * self.cfg.BYTES_PER_BEAT
        expected_data = make_data(frame, beat_index)

        entry = self.memory.get(address)

        # A deterministic design should not need to "wait and see" here.
        # If data is not ready when the schedule says to read it, flag it.
        if entry is None:
            self.errors.append(
                f"Cycle {cycle}: READ BEFORE WRITE at 0x{address:08X} "
                f"(frame {frame}, beat {beat_index})."
            )
            return (
                "READ_RISK",
                Beat(frame, beat_index, -1, address, expected_data),
                "scheduled read occurred before any write to this address",
                None,
            )

        if entry.frame != frame or entry.beat != beat_index:
            self.errors.append(
                f"Cycle {cycle}: WRONG FRAME IN BUFFER at 0x{address:08X}. "
                f"Expected frame {frame} beat {beat_index}, "
                f"found frame {entry.frame} beat {entry.beat}."
            )
            return (
                "READ_RISK",
                Beat(frame, beat_index, -1, address, expected_data),
                "address contains data from a different frame",
                None,
            )

        if entry.data != expected_data:
            self.errors.append(
                f"Cycle {cycle}: DATA MISMATCH at 0x{address:08X}. "
                f"Expected 0x{expected_data:032X}, "
                f"found 0x{entry.data:032X}."
            )

        if entry.consumed:
            self.errors.append(
                f"Cycle {cycle}: DUPLICATE READ at 0x{address:08X} "
                f"(frame {frame}, beat {beat_index})."
            )

        entry.read_cycle = cycle
        gap = cycle - entry.write_cycle

        self.next_read_beat[frame] += 1
        self.completed_reads += 1

        s = self.frame_stats[frame]

        if s.first_read is None:
            s.first_read = cycle
        s.last_read = cycle

        if s.min_write_to_read_gap is None:
            s.min_write_to_read_gap = gap
            s.max_write_to_read_gap = gap
        else:
            s.min_write_to_read_gap = min(s.min_write_to_read_gap, gap)
            s.max_write_to_read_gap = max(s.max_write_to_read_gap, gap)

        beat = Beat(frame, beat_index, -1, address, expected_data)
        return "READ", beat, "", gap

    # -------------------------------------------------------------------------
    # Main simulation
    # -------------------------------------------------------------------------

    def done(self) -> bool:
        all_input_arrived = self.next_input_index == len(self.input_beats)
        no_pending_writes = not self.write_queue
        all_reads_done = self.completed_reads == self.total_beats

        return all_input_arrived and no_pending_writes and all_reads_done

    def run(self) -> None:
        self.validate_config()

        for cycle in range(self.cfg.MAX_CYCLES):
            arrived = self.accept_arrivals(cycle)

            slot, window_id, beat_in_window = slot_for_cycle(cycle, self.cfg)

            operation = "IDLE"
            beat: Optional[Beat] = None
            note = ""
            gap: Optional[int] = None

            if slot == "WRITE_SLOT":
                operation, beat, note = self.do_write(cycle)

            elif slot == "READ_SLOT":
                operation, beat, note, gap = self.do_read(cycle)

            else:
                note = "fixed read/write turnaround gap"

            row = {
                "cycle": cycle,
                "slot": slot,
                "schedule_window": window_id,
                "beat_in_window": beat_in_window,
                "operation": operation,
                "frame": "" if beat is None else beat.frame,
                "stream": "" if beat is None else frame_stream(beat.frame),
                "beat": "" if beat is None else beat.beat,
                "address_hex": "" if beat is None else f"0x{beat.address:08X}",
                "data_hex": "" if beat is None else f"0x{beat.data:032X}",
                "input_arrivals": len(arrived),
                "write_queue_depth": len(self.write_queue),
                "write_to_read_gap": "" if gap is None else gap,
                "note": note,
            }

            self.schedule_rows.append(row)

            if self.done():
                return

        self.errors.append(
            f"Simulation hit MAX_CYCLES={self.cfg.MAX_CYCLES} before completing."
        )

    # -------------------------------------------------------------------------
    # Output
    # -------------------------------------------------------------------------

    def write_outputs(self) -> Path:
        out_dir = Path(self.cfg.OUTPUT_DIR)
        out_dir.mkdir(parents=True, exist_ok=True)

        schedule_path = out_dir / "expected_memory_schedule.csv"
        frame_path = out_dir / "frame_summary.csv"
        report_path = out_dir / "model_report.txt"

        # Cycle-by-cycle expected trace
        if self.schedule_rows:
            with schedule_path.open("w", newline="") as f:
                writer = csv.DictWriter(
                    f, fieldnames=list(self.schedule_rows[0].keys())
                )
                writer.writeheader()
                writer.writerows(self.schedule_rows)

        # Per-frame summary
        with frame_path.open("w", newline="") as f:
            fieldnames = [
                "frame",
                "stream",
                "frame_start_cycle",
                "read_eligible_cycle",
                "first_write",
                "last_write",
                "first_read",
                "last_read",
                "min_write_to_read_gap",
                "max_write_to_read_gap",
            ]

            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()

            for frame in sorted(self.frame_stats):
                s = self.frame_stats[frame]
                writer.writerow(
                    {
                        "frame": s.frame,
                        "stream": s.stream,
                        "frame_start_cycle": s.frame_start_cycle,
                        "read_eligible_cycle": s.read_eligible_cycle,
                        "first_write": s.first_write,
                        "last_write": s.last_write,
                        "first_read": s.first_read,
                        "last_read": s.last_read,
                        "min_write_to_read_gap": s.min_write_to_read_gap,
                        "max_write_to_read_gap": s.max_write_to_read_gap,
                    }
                )

        # Human-readable report
        with report_path.open("w") as f:
            f.write("DDR MEMORY SCHEDULE REFERENCE MODEL\n")
            f.write("=" * 72 + "\n\n")

            f.write("IMPORTANT\n")
            f.write("This is not a MIG/DDR4 protocol simulator.\n")
            f.write(
                "It is an independent reference model of the memory behaviour "
                "the RTL is expected to implement.\n\n"
            )

            f.write("Configuration\n")
            f.write("-" * 72 + "\n")
            for name, value in vars(self.cfg).items():
                f.write(f"{name}: {value}\n")

            f.write("\nResults\n")
            f.write("-" * 72 + "\n")
            f.write(f"Cycles simulated: {len(self.schedule_rows)}\n")
            f.write(f"Total expected beats: {self.total_beats}\n")
            f.write(f"Completed reads: {self.completed_reads}\n")
            f.write(f"Maximum write queue depth: {self.max_write_queue_depth}\n")
            f.write(f"Validation errors: {len(self.errors)}\n")

            gaps = [
                s.min_write_to_read_gap
                for s in self.frame_stats.values()
                if s.min_write_to_read_gap is not None
            ]

            if gaps:
                f.write(
                    "Minimum write-to-read gap across all frames: "
                    f"{min(gaps)} cycles\n"
                )

            f.write("\nPer-frame summary\n")
            f.write("-" * 72 + "\n")

            for frame in sorted(self.frame_stats):
                s = self.frame_stats[frame]
                f.write(
                    f"Frame {s.frame} ({s.stream}): "
                    f"arrival={s.frame_start_cycle}, "
                    f"read_eligible={s.read_eligible_cycle}, "
                    f"first_write={s.first_write}, "
                    f"last_write={s.last_write}, "
                    f"first_read={s.first_read}, "
                    f"last_read={s.last_read}, "
                    f"min_WR_gap={s.min_write_to_read_gap}\n"
                )

            f.write("\nErrors / warnings\n")
            f.write("-" * 72 + "\n")

            if not self.errors:
                f.write("PASS: no reference-model violations detected.\n")
            else:
                for error in self.errors:
                    f.write(f"ERROR: {error}\n")

        return out_dir

    def print_summary(self) -> None:
        print("\nReference model complete")
        print("=" * 72)
        print(f"Data width                 : {self.cfg.DATA_WIDTH_BITS} bits")
        print(f"Bytes per beat             : {self.cfg.BYTES_PER_BEAT}")
        print(f"Frames                     : {self.cfg.NUM_FRAMES}")
        print(f"Beats per frame            : {self.cfg.FRAME_BEATS}")
        print(f"Cycles simulated           : {len(self.schedule_rows)}")
        print(f"Completed reads            : {self.completed_reads}/{self.total_beats}")
        print(f"Max pending write beats    : {self.max_write_queue_depth}")

        gaps = [
            s.min_write_to_read_gap
            for s in self.frame_stats.values()
            if s.min_write_to_read_gap is not None
        ]

        if gaps:
            print(f"Minimum write->read gap    : {min(gaps)} cycles")

        if self.errors:
            print(f"Reference-model violations : {len(self.errors)}")
            print("See model_output/model_report.txt")
        else:
            print("Reference-model violations : 0 (PASS)")

        print("\nGenerated:")
        print("  model_output/expected_memory_schedule.csv")
        print("  model_output/frame_summary.csv")
        print("  model_output/model_report.txt")


# =============================================================================
# MAIN
# =============================================================================

if __name__ == "__main__":
    model = ReferenceModel(CFG)
    model.run()
    model.write_outputs()
    model.print_summary()
