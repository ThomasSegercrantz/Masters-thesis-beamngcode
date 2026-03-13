# Masters-thesis-beamngcode
All necessary code for my Master's thesis

# ACC Thesis Automation (BeamNG.tech + BeamNGpy)

## Requirements
- Windows
- BeamNG.tech v0.38.x installed
- Python 3.10+ installed

## Setup (first time)
```powershell
py -3.11 -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
pip install -r requirements.txt
copy .env.example .env
# edit .env and set BEAMNG_HOME to your BeamNG.tech.exe path