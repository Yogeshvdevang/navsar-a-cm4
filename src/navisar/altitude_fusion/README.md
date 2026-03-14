# Altitude Fusion (AMSL)

This package implements production altitude fusion with explicit frame separation:

- barometer path propagates AMSL changes
- lidar path measures AGL only (tilt-corrected)
- ground elevation state links AMSL and AGL

## Python modules

- `mavlink_input.py`: threaded ATTITUDE + GLOBAL_POSITION_INT reader
- `lidar_input.py`: threaded lidar serial reader (`distance_m[,quality]`)
- `fusion.py`: AMSL-ground-AGL fusion equations and mode logic
- `gps_output.py`: NMEA serial or ArduPilot `GPS_INPUT` output
- `simulation.py`: synthetic scenarios and CSV writer
- `service.py`: runnable companion service loop (30 Hz fusion, 10 Hz output)

## Run service

```bash
python -m navisar.altitude_fusion.service
```

Uses config at `config/altitude_fusion.yaml`.

## Run simulator

```bash
python -m navisar.altitude_fusion.simulation --scenario all --out data/altitude_fusion_sim.csv
```
