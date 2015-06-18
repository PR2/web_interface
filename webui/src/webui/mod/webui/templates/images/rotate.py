
from PIL import Image

for dir in range(0,360):
  im = Image.open("pr2_small.png")
  im2 = im.rotate(dir, Image.BILINEAR, expand=0)
  im2.save("pr2_%d.png" % dir, "png")

