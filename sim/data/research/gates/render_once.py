"""Render one camera topic of a world in this process (TestFixture; ogre2 once per process) and save the
last image: python render_once.py <world.sdf> <topic> <out.png> <seconds>"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[4] / "sim" / "tests"))
import simulate  # noqa: E402,F401  (gzenv environment, private partition)

import cv2  # noqa: E402
import numpy as np  # noqa: E402
from gz.msgs10.image_pb2 import Image  # noqa: E402
from gz.sim8 import TestFixture  # noqa: E402
from gz.transport13 import Node  # noqa: E402

world, topic, out, seconds = sys.argv[1], sys.argv[2], sys.argv[3], float(sys.argv[4])
images = []
node = Node()
node.subscribe(Image, topic, images.append)
fixture = TestFixture(world)
fixture.finalize()
fixture.server().run(True, round(seconds * 1000), False)
if not images:
    print("NO IMAGE")
    sys.exit(1)
img = images[-1]
rgb = np.frombuffer(img.data, np.uint8).reshape(img.height, img.width, 3)
cv2.imwrite(out, cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR))
print("IMAGES", len(images), img.width, img.height)
