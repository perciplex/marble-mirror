from marble_mirror import BallReader
from pathlib import Path


reader = BallReader()


print("integration time:", reader.pixel.integration_time)
print("gain:", reader.pixel.gain)


actual_color = "white"
save_file = Path(f"{actual_color}.txt")

try:
    save_file.unlink()
except:
    pass

while True:
    reader.color

    # with save_file.open('w') as f:
    #    f.write('color_rgb_bytes\tcolor_raw\tcolor_temperature\tlux\n')
    print(reader.all_color_info())
