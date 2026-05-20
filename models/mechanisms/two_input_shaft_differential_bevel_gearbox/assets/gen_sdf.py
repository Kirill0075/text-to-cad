from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from sdf_common import gen_mechanism_sdf


def gen_sdf():
    return gen_mechanism_sdf(Path(__file__).resolve().parents[1])
