from beamngpy import BeamNGpy
from scripts.common import get_settings

def main():
    s = get_settings()
    bng = BeamNGpy(s.host, s.port, home=s.home)
    bng.open(launch=True)
    print("✅ Connected to BeamNG.tech")
    bng.close()

if __name__ == "__main__":
    main()