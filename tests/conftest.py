import sys
from pathlib import Path

# Add project root to sys.path so tests can find quarter_car package
root = Path(__file__).resolve().parent.parent
if str(root) not in sys.path:
    sys.path.insert(0, str(root))
