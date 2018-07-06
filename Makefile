all:
	pio ci examples/logger/main.cpp  --lib src --project-conf platformio.ini
	pio ci examples/motors/main.cpp  --lib src --project-conf platformio.ini