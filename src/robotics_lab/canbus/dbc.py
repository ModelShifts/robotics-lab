"""Synthetic classical-CAN database for the joint loop.

One 11-bit frame per stage. Signals are Intel byte order, unsigned.
``Position`` and ``Velocity`` use a 0.001 scale and a -32.768 offset so a
signed measurement fits in 16 bits. ``Sequence`` is the sample index.
"""

from __future__ import annotations

from pathlib import Path

DBC_TEXT = """\
VERSION ""


NS_ :
	NS_DESC_
	CM_
	BA_DEF_
	BA_
	VAL_
	CAT_DEF_
	CAT_
	FILTER
	BA_DEF_DEF_
	EV_DATA_
	ENVVAR_DATA_
	SGTYPE_
	SGTYPE_VAL_
	BA_DEF_SGTYPE_
	BA_SGTYPE_
	SIG_TYPE_REF_
	VAL_TABLE_
	SIG_GROUP_
	SIG_VALTYPE_
	SIGTYPE_VALTYPE_
	BO_TX_BU_
	BA_DEF_REL_
	BA_REL_
	BA_DEF_DEF_REL_
	BU_SG_REL_
	BU_EV_REL_
	BU_BO_REL_
	SG_MUL_VAL_

BS_:

BU_: SENSOR CTRL ACT

BO_ 256 JointState: 8 SENSOR
 SG_ Position : 0|16@1+ (0.001,-32.768) [-32.768|32.767] "rad" CTRL,ACT
 SG_ Velocity : 16|16@1+ (0.001,-32.768) [-32.768|32.767] "rad/s" CTRL,ACT
 SG_ Sequence : 32|16@1+ (1,0) [0|65535] "" CTRL,ACT

BO_ 512 JointCommand: 8 CTRL
 SG_ Position : 0|16@1+ (0.001,-32.768) [-32.768|32.767] "rad" ACT
 SG_ Sequence : 32|16@1+ (1,0) [0|65535] "" ACT

BO_ 768 JointFeedback: 8 ACT
 SG_ Position : 0|16@1+ (0.001,-32.768) [-32.768|32.767] "rad" CTRL
 SG_ Sequence : 32|16@1+ (1,0) [0|65535] "" CTRL
"""


def write_dbc(path: Path | str) -> Path:
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(DBC_TEXT)
    return path


def load_dbc(path: Path | str | None = None):
    """Load a DBC file, or the built-in synthetic database when ``path`` is omitted."""
    import cantools

    if path is None:
        return cantools.database.load_string(DBC_TEXT, database_format="dbc")
    file = Path(path)
    if not file.is_file():
        raise FileNotFoundError(file)
    return cantools.database.load_file(file)
