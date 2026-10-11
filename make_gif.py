"""Convert outputs/videos/demo.mp4 into a small GIF for the README."""
import imageio.v2 as imageio
from PIL import Image

SRC = "outputs/videos/demo.mp4"
DST = "docs/images/demo.gif"

frames = []
for i, frame in enumerate(imageio.get_reader(SRC)):
    if i % 2:                                   # keep every 2nd frame
        continue
    img = Image.fromarray(frame).resize((672, 352))
    frames.append(img.convert("P", palette=Image.ADAPTIVE, colors=128))

frames[0].save(DST, save_all=True, append_images=frames[1:],
               duration=100, loop=0, optimize=True)
print(f"Saved {DST} ({len(frames)} frames)")